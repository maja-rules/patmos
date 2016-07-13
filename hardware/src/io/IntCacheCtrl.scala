/*
   Copyright 2013 Technical University of Denmark, DTU Compute.
   All rights reserved.

   This file is part of the time-predictable VLIW processor Patmos.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

      1. Redistributions of source code must retain the above copyright notice,
         this list of conditions and the following disclaimer.

      2. Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER ``AS IS'' AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
   NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   The views and conclusions contained in the software and documentation are
   those of the authors and should not be interpreted as representing official
   policies, either expressed or implied, of the copyright holder.
 */
/*
 * value of mode is 0 per default
 * can be set to 1 or 0 by writing
 *
 * Authors: Wolfgang Puffitsch (wpuffitsch@gmail.com)
 *          Maja Lund
 *
 */

package io

import Chisel._
import Node._

import ocp._

object IntCacheCtrl extends DeviceObject {
  var bitCount=1

  def init(params: Map[String, String]) = { }

  def create(params: Map[String, String]) : IntCacheCtrl = {
    Module(new IntCacheCtrl())
  }

  trait Pins {
    val intCacheCtrlPins = new Bundle() {
      val mode = Bits(OUTPUT, bitCount)
    }
  }
}

class IntCacheCtrl() extends CoreDevice() {

  override val io = new CoreDeviceIO() with IntCacheCtrl.Pins

  val modeReg = Reg(init = Bits(0, 1))

  // Default response
  val respReg = Reg(init = OcpResp.NULL)
  respReg := OcpResp.NULL

  // Write to IntCacheCtrl
  when(io.ocp.M.Cmd === OcpCmd.WR) {
    respReg := OcpResp.DVA
    modeReg := io.ocp.M.Data(0, 0)
  }

  // Read current state of IntCacheCtrl
  when(io.ocp.M.Cmd === OcpCmd.RD) {
    respReg := OcpResp.DVA
  }

  // Connections to master
  io.ocp.S.Resp := respReg
  io.ocp.S.Data := modeReg

  // Connection to pins
  io.intCacheCtrlPins.mode := Reg(next = modeReg)
  io.intCacheMode := modeReg(0) === Bits(1)
}
