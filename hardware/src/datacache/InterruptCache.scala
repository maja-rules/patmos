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
 * An interrupt cache, based on the direct-mapped cache
 *
 * Authors: Martin Schoeberl (martin@jopdesign.com)
 *          Wolfgang Puffitsch (wpuffitsch@gmail.com)
 *          Maja Lund
 */

package datacache

import Chisel._
import Node._

import patmos.Constants._
import patmos.DataCachePerf
import patmos.MemBlock
import patmos.MemBlockIO

import ocp._

class InterruptCache(size: Int, lineSize: Int) extends Module {
  val io = new Bundle {
    val master = new OcpCoreSlavePort(ADDR_WIDTH, DATA_WIDTH)
    val slave = new OcpBurstMasterPort(ADDR_WIDTH, DATA_WIDTH, lineSize/4)
    val invalidate = Bool(INPUT)
    val perf = new DataCachePerf()
    val intCacheInt = Bool(OUTPUT)
  }

  io.perf.hit := Bool(false)
  io.perf.miss := Bool(false)
  io.intCacheInt := Bool(false)

  val addrBits = log2Up(size / BYTES_PER_WORD)
  val lineBits = log2Up(lineSize)

  val tagWidth = ADDR_WIDTH - addrBits - 2
  val tagCount = size / lineSize

  // Register signals from master
  val masterReg = Reg(io.master.M)
  masterReg := io.master.M

  // Generate memories
  val tagMem = MemBlock(tagCount, tagWidth)
  val tagVMem = Vec.fill(tagCount) { Reg(init = Bool(false)) }
  val mem = new Array[MemBlockIO](BYTES_PER_WORD)
  for (i <- 0 until BYTES_PER_WORD) {
    mem(i) = MemBlock(size / BYTES_PER_WORD, BYTE_WIDTH).io
  }

  val missReg = Reg(init = Bits(0,32))
  val MemRes = Vec.fill(4) { Reg(init = Bits(0,32)) }

  //choose different tag address for direct tag commands
  val tagCheck =Bits("h90000800")<=io.master.M.Addr && io.master.M.Addr <= Bits("h900009FF")
  val tagAddr = Mux(tagCheck,io.master.M.Addr(8,2),io.master.M.Addr(addrBits + 1, lineBits))
  val tagCheckReg = Reg(next = tagCheck)

  val tag = tagMem.io(tagAddr)
  val tagV = Reg(next = tagVMem(tagAddr))
  val tagValid = tagV && tag === Cat(masterReg.Addr(ADDR_WIDTH-1, addrBits+2))

  val wrAddrReg = Reg(Bits(width = addrBits))
  val wrDataReg = Reg(Bits(width = DATA_WIDTH))

  wrAddrReg := io.master.M.Addr(addrBits + 1, 2)
  wrDataReg := io.master.M.Data

  val cacheCheck = Bits("h90000000")<=masterReg.Addr && masterReg.Addr <= Bits("h900007FF")

  // Write to cache on write hit or on direct cache write
  val stmsk = Mux(masterReg.Cmd === OcpCmd.WR, masterReg.ByteEn,  Bits("b0000"))
  for (i <- 0 until BYTES_PER_WORD) {
    mem(i) <= ((tagValid && stmsk(i)) || (cacheCheck && stmsk(i)), wrAddrReg,
               wrDataReg(BYTE_WIDTH*(i+1)-1, BYTE_WIDTH*i))
  }

  val memCheck = Bits("h90000A00")===masterReg.Addr
  val valCheck = Bits("h90000A04")<=masterReg.Addr && masterReg.Addr <= Bits("h90000A10")

  // Read from cache
  val rdData = mem.map(_(io.master.M.Addr(addrBits + 1, 2))).reduceLeft((x,y) => y ## x)

  // Return data on a hit or when address is one of the designated cache addresses
  io.master.S.Data := rdData
  when(valCheck){
    io.master.S.Data := MemRes(masterReg.Addr(4,2)-UInt(1))
  }
  when(tagCheckReg && tagV){
    io.master.S.Data := (UInt(1)<<UInt(21))|tag
  }.otherwise{
    when(tagCheckReg && !tagV){
      io.master.S.Data := tag
    }
  }
  io.master.S.Resp := Mux((tagValid && masterReg.Cmd === OcpCmd.RD)||
                            (cacheCheck && masterReg.Cmd === OcpCmd.RD)||
                              (cacheCheck && masterReg.Cmd === OcpCmd.WR)||
                                (tagCheckReg && masterReg.Cmd === OcpCmd.RD)||
                                  (tagCheckReg && masterReg.Cmd === OcpCmd.WR)||
                                    (valCheck && masterReg.Cmd === OcpCmd.RD),
                                      OcpResp.DVA, OcpResp.NULL)

  // State machine for burst reads
  val idle :: fill :: hold :: Nil = Enum(UInt(), 3)
  val stateReg = Reg(init = idle)

  // Default values
  io.slave.M.Cmd := OcpCmd.IDLE
  io.slave.M.Addr := Bits(0)
  io.slave.M.Data := Bits(0)
  io.slave.M.DataValid := Bits(0)
  io.slave.M.DataByteEn := Bits(0)

  // Record a hit
  when(tagValid && masterReg.Cmd === OcpCmd.RD) {
    io.perf.hit := Bool(true)
  }

  // Start handling a miss
  when((!tagValid && masterReg.Cmd === OcpCmd.RD) && !cacheCheck && !tagCheck
          && !tagCheckReg && !memCheck && stateReg === idle && !valCheck) {  
    io.intCacheInt := Bool(true)
    missReg := Cat(masterReg.Addr(31,4),Bits(0,4))
    io.perf.miss := Bool(true)
  }

  val tagWrAddr = masterReg.Addr(8,2)
  val tagWrData = masterReg.Data(20,0)
  
  //set tag valid on direct tag write when appropriate
  when(tagCheckReg && masterReg.Data(21) && masterReg.Cmd === OcpCmd.WR){
    tagVMem(tagWrAddr) := Bool(true)
  }.otherwise{ 
    when(tagCheckReg && !masterReg.Data(21) && masterReg.Cmd === OcpCmd.WR){
      tagVMem(tagWrAddr) := Bool(false)
    }
  }

  //save tag on direct tag write
  tagMem.io <= (tagCheckReg && masterReg.Cmd === OcpCmd.WR, tagWrAddr,tagWrData)

  //no writes allowed to these addresses
  when((memCheck || valCheck) && masterReg.Cmd === OcpCmd.WR){
    io.master.S.Resp := OcpResp.ERR
  }

  val burstCntReg = Reg(init = UInt(0, lineBits-2))

  //start bundle read
  when(memCheck && masterReg.Cmd === OcpCmd.RD){
    io.slave.M.Cmd := OcpCmd.RD
    io.slave.M.Addr := missReg
    when(io.slave.S.CmdAccept === Bits(1)) {
      stateReg := fill
    }
    .otherwise {
      stateReg := hold
    }
  }

  // Hold read command
  when(stateReg === hold) {
    io.slave.M.Cmd := OcpCmd.RD
    io.slave.M.Addr := missReg
    when(io.slave.S.CmdAccept === Bits(1)) {
      stateReg := fill
    }
    .otherwise {
      stateReg := hold
    }
  }

  // Wait for response
  when(stateReg === fill) {
    when(io.slave.S.Resp =/= OcpResp.NULL) {
      MemRes(burstCntReg) := io.slave.S.Data

      when(burstCntReg === UInt(lineSize/4-1)) {
        io.master.S.Data := missReg
        io.master.S.Resp := OcpResp.DVA
        stateReg := idle
      }
      burstCntReg := burstCntReg + UInt(1)
    }
  }

  // reset valid bits
  when (io.invalidate) {
    tagVMem.map(_ := Bool(false))
  }
}
