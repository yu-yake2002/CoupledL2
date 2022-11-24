/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  * http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

package coupledL2

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.SetAssocLRU
import coupledL2.utils._
import chipsalliance.rocketchip.config.Parameters

class MetaEntry(implicit p: Parameters) extends L2Bundle {
  val dirty = Bool()
  val state = UInt(stateBits.W)
  val clients = UInt(clientBits.W)  // valid-bit of clients
  // TODO: record specific state of clients instead of just 1-bit
  // TODO: record prefetch info

  def =/=(entry: MetaEntry): Bool = {
    this.asUInt =/= entry.asUInt
  }
}

object MetaEntry {
  def apply()(implicit p: Parameters) = {
    val init = WireInit(0.U.asTypeOf(new MetaEntry))
    init
  }
  def apply(dirty: Bool, state: UInt, clients: UInt)(implicit p: Parameters) = {
    val entry = Wire(new MetaEntry)
    entry.dirty := dirty
    entry.state := state
    entry.clients := clients
    entry
  }
}

class DirRead(implicit p: Parameters) extends L2Bundle {
  val tag = UInt(tagBits.W)
  val set = UInt(setBits.W)
  val source = UInt(sourceIdBits.W)
  val replacerInfo = new ReplacerInfo()
}

class DirResult(implicit p: Parameters) extends L2Bundle {
  val hit = Bool()
  val tag = UInt(tagBits.W)
  val set = UInt(setBits.W)
  val way = UInt(wayBits.W)  // hit way or victim way
  val meta = new MetaEntry()
  val error = Bool()
}

class MetaWrite(implicit p: Parameters) extends L2Bundle {
  val set = UInt(setBits.W)
  val wayOH = UInt(cacheParams.ways.W)
  val wmeta = new MetaEntry
}

class TagWrite(implicit p: Parameters) extends L2Bundle {
  val set = UInt(setBits.W)
  val way = UInt(wayBits.W)
  val wtag = UInt(tagBits.W)
}

class Directory(implicit p: Parameters) extends L2Module with DontCareInnerLogic {

  val io = IO(new Bundle() {
    val read = Flipped(ValidIO(new DirRead))
    val resp = ValidIO(new DirResult)
    val metaWReq = Flipped(ValidIO(new MetaWrite))
    val tagWReq = Flipped(ValidIO(new TagWrite))
  })

  def invalid_way_sel(metaVec: Seq[MetaEntry], repl: UInt) = {
    val invalid_vec = metaVec.map(_.state === MetaData.INVALID)
    val has_invalid_way = Cat(invalid_vec).orR
    val way = ParallelPriorityMux(invalid_vec.zipWithIndex.map(x => x._1 -> x._2.U(wayBits.W)))
    (has_invalid_way, way)
  }

  val sets = cacheParams.sets
  val ways = cacheParams.ways
  val tag_wen = io.tagWReq.valid
  val dir_wen = io.metaWReq.valid

  val tagArray = Module(new SRAMTemplate(UInt(tagBits.W), sets, ways, singlePort = true))
  val metaArray = Module(new SRAMTemplate(new MetaEntry, sets, ways, singlePort = true))
  val tagRead = Wire(Vec(ways, UInt(tagBits.W)))
  val metaRead = Wire(Vec(ways, new MetaEntry()))

  val reqValidReg = RegNext(io.read.fire, false.B)

  tagArray.io.r <> DontCare
  tagArray.io.w <> DontCare
  metaArray.io.r <> DontCare
  metaArray.io.w <> DontCare

  // Tag R/W
  tagRead := tagArray.io.r(io.read.fire, io.read.bits.set).resp.data
  tagArray.io.w(
    tag_wen,
    io.tagWReq.bits.wtag,
    io.tagWReq.bits.set,
    UIntToOH(io.tagWReq.bits.way)
  )

  // Meta R/W
  metaRead := metaArray.io.r(io.read.fire, io.read.bits.set).resp.data
  metaArray.io.w(
    dir_wen,
    io.metaWReq.bits.wmeta,
    io.metaWReq.bits.set,
    io.metaWReq.bits.wayOH
  )

  // Generate response signals
  /* stage 0: io.read.fire, access Tag/Meta
     stage 1: get Tag/Meta, calculate hit/way
     stage 2: output latched hit/way and chosen meta/tag by way
  */
  // TODO: how about moving hit/way calculation to stage 2? Cuz SRAM latency can be high under high frequency
  val reqReg = RegEnable(io.read.bits, enable = io.read.fire)
  val tagMatchVec = tagRead.map(_ (tagBits - 1, 0) === reqReg.tag)
  val metaValidVec = metaRead.map(_.state =/= MetaData.INVALID)
  val hitVec = tagMatchVec.zip(metaValidVec).map(x => x._1 && x._2)
  val hitWay = OHToUInt(hitVec)
  val replaceWay = 0.U(wayBits.W)  // TODO: add replacer logic
  val (inv, invalidWay) = invalid_way_sel(metaRead, replaceWay)
  val chosenWay = Mux(inv, invalidWay, replaceWay)

//====================Begin=======================================


  class lfsrRandom(numBits: Int = 12) extends Module{
    val io = IO(new Bundle{
      val update_en = Input(Bool())
      val rand_num  = Output(UInt(numBits.W))
    })

    val lfsr0Bits = numBits - 1
    val lfsr0_reg = RegInit( VecInit( Seq.fill(lfsr0Bits)(false.B) ))
    val lfsr1_reg = RegInit( true.B )

    when(io.update_en){
      for(i <- 0 until lfsr0Bits){
        if(i == 0){
          lfsr0_reg(i) := lfsr1_reg
        }
        else if(i == 2 || i == 3 || i == (numBits/2)){
          lfsr0_reg(i) := lfsr0_reg(i-1) ^ lfsr1_reg
        }
        else{
          lfsr0_reg(i) := lfsr0_reg(i-1) 
        }
      }
      lfsr1_reg := lfsr0_reg(lfsr0Bits - 1)
    }

    io.rand_num := Cat(lfsr0_reg.asUInt, lfsr1_reg)
  }


  val log2WayCount = Log2(wayBits)
  val numBits      = 4 * log2WayCount

  val repl_lfsr_en = Wire(Bool())

  val repl_lfsr      = Module(new lfsrRandom(numBits))
  val repl_plru_reg  = RegInit( VecInit( Seq.fill(7)(false.B) ))

  val repl_lfsr_victim_way = Wire( Vec(8, Bool() ))
  val repl_plru_victim_way = Wire( Vec(8, Bool() ))

  val l1_valid_vec  = metaRead.map{x => x.state === MetaData.INVALID}
  val l2_valid_vec  = metaRead.map{x => x.clients}

  val all_l1_valid_en = l1_valid_vec.asUInt.andR

  val rand_way_0  = Wire( Vec(wayBits, Bool()) )
  val rand_way_1  = Wire( Vec(wayBits, Bool()) )
  val rand_way_2  = Wire( Vec(wayBits, Bool()) )
  val order_way   = Wire( Vec(wayBits, Bool()) )

  val update_lfsr_en = io.read.valid | io.metaWReq.valid | io.tagWReq.valid
  repl_lfsr.io.update_en := update_lfsr_en
  val lfsr_rand_num = repl_lfsr.io.rand_num

  val sel_rand_way_0 = ~(l1_valid_vec.asUInt & rand_way_0.asUInt).orR
  val sel_rand_way_1 = ~(l1_valid_vec.asUInt & rand_way_1.asUInt).orR
  val sel_rand_way_2 = ~(l1_valid_vec.asUInt & rand_way_2.asUInt).orR
  val sel_order_way  = ~l1_valid_vec.asUInt.andR

  // log2WayCout = (wayCount == 8)  ? 3 :
  //               (wayCount == 16) ? 4 :
  // rand_way_0 = lfsr_rand_num(2, 0) or lfsr_rand_num(3,  0)
  // rand_way_1 = lfsr_rand_num(5, 3) or lfsr_rand_num(7,  4)
  // rand_way_2 = lfsr_rand_num(8, 6) or lfsr_rand_num(11, 8)
  rand_way_0 := UIntToOH(lfsr_rand_num(1*log2WayCount - 1, 0*log2WayCount)) 
  rand_way_1 := UIntToOH(lfsr_rand_num(2*log2WayCount - 1, 1*log2WayCount))
  rand_way_2 := UIntToOH(lfsr_rand_num(3*log2WayCount - 1, 2*log2WayCount))

  val order_way_index = PriorityEncoder(~l1_valid_vec.asUInt)
  order_way := UIntToOH(order_way_index)


  repl_lfsr_victim_way = Mux(sel_rand_way_0,   rand_way_0,
                         Mux(sel_rand_way_1,   rand_way_1,
                         Mux(sel_rand_way_2,   rand_way_2,
                         Mux(sel_order_way,    order_way,
                                               1.U(wayCount.W)
                            ))))



  val has_l2_invalid_en = ~l2_valid_vec.asUInt.andR
  val l2_invalid_index  = PriorityEncoder(~l2_valid_vec.asUInt)
  val l2_invalid_way    = UIntToOH(l2_invalid_index)

  val plru_way_sel = Wire( Vec(7, Bool()))

  plru_way_sel(0) = ( ~repl_plru_reg(0) & ~l1_valid_vec(7, 4).asUInt.andR ) |
                    ( ~l1_valid_vec(7, 4).asUInt.andR & l1_valid_vec(3, 0).asUInt.andR)

  plru_way_sel(1) = ( ~repl_plru_reg(1) & ~l1_valid_vec(3, 2).asUInt.andR ) |
                    ( ~l1_valid_vec(3, 2).asUInt.andR & l1_valid_vec(1, 0).asUInt.andR)

  plru_way_sel(2) = ( ~repl_plru_reg(2) & ~l1_valid_vec(7, 6).asUInt.andR ) |
                    ( ~l1_valid_vec(7, 6).asUInt.andR & l1_valid_vec(5, 4).asUInt.andR)

  plru_way_sel(3) = ( ~repl_plru_reg(3) & ~l1_valid_vec(1) ) |
                    ( ~l1_valid_vec(1) & l1_valid_vec(0) )

  plru_way_sel(4) = ( ~repl_plru_reg(4) & ~l1_valid_vec(3) ) |
                    ( ~l1_valid_vec(3) & l1_valid_vec(2) )

  plru_way_sel(5) = ( ~repl_plru_reg(5) & ~l1_valid_vec(5) ) |
                    ( ~l1_valid_vec(5) & l1_valid_vec(4) )

  plru_way_sel(6) = ( ~repl_plru_reg(6) & ~l1_valid_vec(7) ) |
                    ( ~l1_valid_vec(7) & l1_valid_vec(6) )

  plru_way(0) := ~plru_way_sel(0) & ~plru_way_sel(1) & ~plru_way_sel(3)
  plru_way(1) := ~plru_way_sel(0) & ~plru_way_sel(1) &  plru_way_sel(3)
  plru_way(2) := ~plru_way_sel(0) &  plru_way_sel(1) & ~plru_way_sel(4)
  plru_way(3) := ~plru_way_sel(0) &  plru_way_sel(1) &  plru_way_sel(4)
  plru_way(4) :=  plru_way_sel(0) & ~plru_way_sel(2) & ~plru_way_sel(5)
  plru_way(5) :=  plru_way_sel(0) & ~plru_way_sel(2) &  plru_way_sel(5)
  plru_way(6) :=  plru_way_sel(0) &  plru_way_sel(2) & ~plru_way_sel(6)
  plru_way(7) :=  plru_way_sel(0) &  plru_way_sel(2) &  plru_way_sel(6)
  
  repl_plru_victim_way = Mux(has_l2_invalid_en, l2_invalid_way, plru_way)
  
  val repl_victim_way = Mux(all_l1_valid_en, rand_way_0,
                        Mux(repl_lfsr_en     repl_lfsr_victim_way,
                                             repl_plru_victim_way
                            ))


  /***
    repl_plru_reg 更新的分析：
                                    是否更新       更新的内容(即写入reg的内容)
            读 Directory 并且 Hit      yes           Hit 的 way
            读 Directory 并且 Miss     no             ----
            写 Directory              yes            写入的 way


    替换算法使用动态的替换算法，
            当 repl_lfsr_en = 1，表示使用的随机替换算法
            当 repl_lfsr_en = 0, 表示使用的 plru 替换算法          


    接下来的安排是：
          整体的替换算法基本是写完了，剩下没有写的地方是 repl_plru_reg 的更新
          访问 Directory 的时候，需要设置 repl_plru_reg 的写使能信号，写入数据
          把这些信号传入 mainpipeline，等合适的阶段再传入 Directory 中，写入 repl_plru_reg 中，完成更新
  ***/




//====================End=======================================
  val hit_s1 = Cat(hitVec).orR
  val way_s1 = Mux(hit_s1, hitWay, chosenWay)

  val reqValid_s2 = reqValidReg
  val hit_s2 = RegEnable(hit_s1, false.B, reqValidReg)
  val way_s2 = RegEnable(way_s1, 0.U, reqValidReg)
  val metaAll_s2 = RegEnable(metaRead, reqValidReg)
  val tagAll_s2 = RegEnable(tagRead, reqValidReg)
  val meta_s2 = metaAll_s2(way_s2)
  val tag_s2 = tagAll_s2(way_s2)
  val set_s2 = RegEnable(reqReg.set, reqValidReg)

  io.resp.valid      := reqValid_s2
  io.resp.bits.hit   := hit_s2
  io.resp.bits.way   := way_s2
  io.resp.bits.meta  := meta_s2
  io.resp.bits.tag   := tag_s2
  io.resp.bits.set   := set_s2
  io.resp.bits.error := false.B  // depends on ECC

  dontTouch(io)
  dontTouch(metaArray.io)
  dontTouch(tagArray.io)

}
