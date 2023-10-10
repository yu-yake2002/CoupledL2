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
import utility._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink._
import coupledL2.utils.XSPerfAccumulate
import huancun.DirtyKey

class SourceC(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle() {
    val in = Flipped(DecoupledIO(new TaskWithData()))
    val out = DecoupledIO(new TLBundleC(edgeOut.bundle))
    val resp = Output(new RespBundle)
  })

  def toTLBundleC(task: TaskBundle, data: UInt = 0.U) = {
    val c = Wire(new TLBundleC(edgeOut.bundle))
    c.opcode := task.opcode
    c.param := task.param
    c.size := offsetBits.U
    c.source := task.mshrId
    c.address := Cat(task.tag, task.set, task.off)
    c.data := data
    c.corrupt := false.B
    c.user.lift(utility.ReqSourceKey).foreach(_ := task.reqSource)
    c.echo.lift(DirtyKey).foreach(_ := task.dirty)
    c
  }

  // We must keep SourceC FIFO, so a queue is used
  // TODO: may be not enough / excessive? WARNING
  // it is better to add back pressure logic from SourceC
  val queue = Module(new Queue(new TaskWithData(), entries = mshrsAll))

  // dequeued entry stored in buf
  val bufValid = RegInit(false.B)
  val buf = RegInit(0.U.asTypeOf(new TaskWithData()))

  queue.io.enq <> io.in
  queue.io.deq.ready := io.out.ready && !bufValid

  val deqValid = queue.io.deq.valid
  val deqTask = queue.io.deq.bits.task
  val deqData = queue.io.deq.bits.data.asTypeOf(Vec(beatSize, new DSBeat))

  // if deqTask has data, send the first beat directly and save the remaining beat in buf
  when(deqValid && io.out.ready && !bufValid && deqTask.opcode(0)) {
    bufValid := true.B
    buf.task := deqTask
    buf.data := deqData(1) // TODO: this only applies to beatSize = 2
  }
  when(bufValid && io.out.ready) {
    bufValid := false.B
  }

  io.out.valid := bufValid || deqValid
  io.out.bits := Mux(
    bufValid,
    toTLBundleC(buf.task, buf.data.data),
    toTLBundleC(deqTask, deqData(0).data)
  )

  assert(io.in.ready, "SourceC should never be full") // WARNING

  // ========== Misc ============
  val (first, last, done, count) = edgeOut.count(io.out)
  val isRelease = io.out.bits.opcode === TLMessages.Release
  val isReleaseData = io.out.bits.opcode === TLMessages.ReleaseData

  // TODO: resp from SourceC indicating w_release_sent is deprecated, unused in MSHR
  io.resp.valid := io.out.fire && first && (isRelease || isReleaseData)
  io.resp.mshrId := io.out.bits.source
  io.resp.set := parseFullAddress(io.out.bits.address)._2
  io.resp.tag := parseFullAddress(io.out.bits.address)._1
  io.resp.respInfo := 0.U.asTypeOf(new RespInfoBundle)
}