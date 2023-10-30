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
import coupledL2.utils.{BankedSRAM, SRAMTemplate}
import utility.RegNextN
import org.chipsalliance.cde.config.Parameters

class DSRequest(implicit p: Parameters) extends L2Bundle {
  val way = UInt(wayBits.W)
  val set = UInt(setBits.W)
  val wen = Bool()
}

class DSBeat(implicit p: Parameters) extends L2Bundle {
  val data = UInt((beatBytes * 8).W)
}

class DSBlock(implicit p: Parameters) extends L2Bundle {
  val data = UInt((blockBytes * 8).W)

  // WARNING:TODO: check this
  def toBeats: Vec[DSBeat] = Reverse(data).asTypeOf(Vec(beatSize, new DSBeat))
}

class DataStorage(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    // there is only 1 read or write request in the same cycle,
    // so only 1 req port is necessary
    val req = Flipped(ValidIO(new DSRequest))
    val rdata = Output(new DSBlock)
    val wdata = Input(new DSBlock)
  })

  // convert SRAM size from 4096 * 512 to multiple 1024 * 128
  val dataSplit = 4 // split a cache block
  val indexSplit = 4 // split SRAM index
  val dataSplitWidth = blockBytes * 8 / dataSplit

  val arrayIdx = Cat(io.req.bits.way, io.req.bits.set)
  val wen = io.req.valid && io.req.bits.wen
  val ren = io.req.valid && !io.req.bits.wen

  val array = Seq.fill(dataSplit) {
    Module(new BankedSRAM(
      gen = UInt(dataSplitWidth.W),
      sets = blocks,
      ways = 1,
      n = indexSplit
    ))
  }

  for (i <- 0 until dataSplit) {
    array(i).io.w.apply(wen, io.wdata.data(dataSplitWidth * (i + 1) - 1, dataSplitWidth * i), arrayIdx, 1.U)
    array(i).io.r.apply(ren, arrayIdx)
  }

  io.rdata := Cat(array.map(_.io.r.resp.data(0)).reverse).asTypeOf(new DSBlock)
  // TODO: timing: we should not use reg here, instead set this as multicycle path
  // s3 read, s4 pass and s5 to destination
}
