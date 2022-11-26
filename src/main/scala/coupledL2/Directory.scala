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


class plru8 extends Module{
  val io = IO(new Bundle{
    val hit_way  = Flipped(Valid(UInt(8.W)))
    val plru_reg = Output(UInt(7.W))
  })

  /***
    way7     way6    way5     way4    way3     way2    way1     way0
      \      /         \      /         \      /         \      /   
      1    0           1    0           1    0           1    0    
        \  /             \  /             \  /             \  /     
      reg[6]           reg[5]           reg[4]           reg[3]    
          \               /                 \               /  
          1             0                   1             0   
          -----\   /----                    -----\   /----    
              reg[2]                            reg[1]        
                  \                                /
                  1                              0
                    -------------\    /------------
                                reg[0]

        reg[x] = 1, accessed the left node
        reg[x] = 0, accessed the right node
        
    Example:
        reg[0] = 1, accessed way7 ~ way4
        reg[2] = 0, accessed way5 ~ way4
        reg[5] = 1, accessed way5
  ***/

  val plru_reg_en = io.hit_way.valid
  val hit_way     = io.hit_way.bits

  val plru_reg = RegInit( VecInit( Seq.fill(7)(false.B) ))

  when(plru_reg_en){
    plru_reg(0) :=      hit_way(7, 4).orR
    plru_reg(1) := Mux( hit_way(3, 0).orR,  hit_way(3, 2).orR,  plru_reg(1))
    plru_reg(2) := Mux( hit_way(7, 4).orR,  hit_way(7, 6).orR,  plru_reg(2))
    plru_reg(3) := Mux( hit_way(1, 0).orR,  hit_way(1),         plru_reg(3))
    plru_reg(4) := Mux( hit_way(3, 2).orR,  hit_way(3),         plru_reg(4))
    plru_reg(5) := Mux( hit_way(5, 4).orR,  hit_way(5),         plru_reg(5))
    plru_reg(6) := Mux( hit_way(7, 6).orR,  hit_way(7),         plru_reg(6))
  }

  io.plru_reg := plru_reg.asUInt
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
//====================Begin=======================================


  val l2_invalid_vec = metaRead.map(x => x.state === MetaData.INVALID)

  val has_l2_invalid_way = Cat(l2_invalid_vec).orR
  val l2_invalid_index   = PriorityEncoder(Cat(l2_invalid_vec))
  val l2_invalid_way     = UIntToOH(l2_invalid_index)


  val l1_valid_vec   = metaRead.map(x => x.clients)

  val plru8    = Module(new plru8())
  val plru_reg = plru8.io.plru_reg

  val l1_valid_way_3_0_en = Cat(l1_valid_vec)(3, 0).andR
  val l1_valid_way_7_4_en = Cat(l1_valid_vec)(7, 4).andR
  val l1_valid_way_1_0_en = Cat(l1_valid_vec)(1, 0).andR
  val l1_valid_way_3_2_en = Cat(l1_valid_vec)(3, 2).andR
  val l1_valid_way_5_4_en = Cat(l1_valid_vec)(5, 4).andR
  val l1_valid_way_7_6_en = Cat(l1_valid_vec)(7, 6).andR

  val l1_valid_way_0_en = l1_valid_vec(0)
  val l1_valid_way_1_en = l1_valid_vec(1)
  val l1_valid_way_2_en = l1_valid_vec(2)
  val l1_valid_way_3_en = l1_valid_vec(3)
  val l1_valid_way_4_en = l1_valid_vec(4)
  val l1_valid_way_5_en = l1_valid_vec(5)
  val l1_valid_way_6_en = l1_valid_vec(6)
  val l1_valid_way_7_en = l1_valid_vec(7)

  val plru_sel = Wire(Vec(7, Bool()))
  plru_sel(0) := ( ~plru_reg(0) & ~l1_valid_way_3_0_en) |
                 ( ~l1_valid_way_7_4_en & l1_valid_way_3_0_en)
  plru_sel(1) := ( ~plru_reg(1) & ~l1_valid_way_1_0_en) |
                 ( ~l1_valid_way_3_2_en & l1_valid_way_1_0_en)
  plru_sel(2) := ( ~plru_reg(2) & ~l1_valid_way_5_4_en) |
                 ( ~l1_valid_way_7_6_en & l1_valid_way_5_4_en)
  plru_sel(3) := ( ~plru_reg(3) & ~l1_valid_way_0_en) |
                 ( ~l1_valid_way_1_en & l1_valid_way_0_en)
  plru_sel(4) := ( ~plru_reg(4) & ~l1_valid_way_2_en) |
                 ( ~l1_valid_way_3_en & l1_valid_way_2_en)
  plru_sel(5) := ( ~plru_reg(5) & ~l1_valid_way_4_en) |
                 ( ~l1_valid_way_5_en & l1_valid_way_4_en)
  plru_sel(6) := ( ~plru_reg(6) & ~l1_valid_way_6_en) |
                 ( ~l1_valid_way_7_en & l1_valid_way_6_en)

  val plru_way = Wire(Vec(8, Bool()))
  plru_way(0) := ~plru_sel(0) & ~plru_sel(1) & ~plru_sel(3)
  plru_way(1) := ~plru_sel(0) & ~plru_sel(1) &  plru_sel(3)
  plru_way(2) := ~plru_sel(0) &  plru_sel(1) & ~plru_sel(4)
  plru_way(3) := ~plru_sel(0) &  plru_sel(1) &  plru_sel(4)
  plru_way(4) :=  plru_sel(0) & ~plru_sel(2) & ~plru_sel(5)
  plru_way(5) :=  plru_sel(0) & ~plru_sel(2) &  plru_sel(5)
  plru_way(6) :=  plru_sel(0) &  plru_sel(2) & ~plru_sel(6)
  plru_way(7) :=  plru_sel(0) &  plru_sel(2) &  plru_sel(6)

  val repl_victim_way = Mux(has_l2_invalid_way, l2_invalid_way, plru_way.asUInt)

  // update plru data
  val plru_update_en   = io.metaWReq.valid & (io.metaWReq.bits.wmeta.state =/= MetaData.INVALID)
  val plru_update_data = io.metaWReq.bits.wayOH
  plru8.io.hit_way.valid := plru_update_en
  plru8.io.hit_way.bits  := plru_update_data

//====================End=======================================
  val hit_s1 = Cat(hitVec).orR
  val way_s1 = Mux(hit_s1, hitWay, repl_victim_way)

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
