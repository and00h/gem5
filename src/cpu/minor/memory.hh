/*
 * Copyright (c) 2013-2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *
 *  All the fun of executing instructions from Decode and sending branch/new
 *  instruction stream info. to Fetch1.
 */

#ifndef __CPU_MINOR_MEMORY_HH__
#define __CPU_MINOR_MEMORY_HH__

#include <vector>

#include "base/named.hh"
#include "base/types.hh"
#include "cpu/minor/buffers.hh"
#include "cpu/minor/cpu.hh"
#include "cpu/minor/func_unit.hh"
#include "cpu/minor/lsq.hh"
#include "cpu/minor/pipe_data.hh"
#include "cpu/minor/scoreboard.hh"

namespace gem5
{

  GEM5_DEPRECATED_NAMESPACE(Minor, minor);
  namespace minor
  {

    /** Memory stage.  Everything apart from fetching and decoding instructions.
     *  The LSQ lives here too. */
    class Memory : public Named
    {
    protected:
      Latch<ForwardInstData>::Output inp;
      Latch<ForwardInstData>::Input out;

      /** Input port carrying stream changes to Fetch1 */
      Latch<BranchData>::Input branch_out;
      Latch<BranchData>::Output branch_in;

      MinorCPU &cpu;
      unsigned int issueLimit;
      unsigned int memoryIssueLimit;
      unsigned int commitLimit;
      unsigned int memoryCommitLimit;

      std::vector<InputBuffer<ForwardInstData>> &nextStageReserve;
      unsigned int outputWidth;

      bool processMoreThanOneInput;
      bool setTraceTimeOnCommit;

    public:
      std::vector<InputBuffer<ForwardInstData>> inputBuffer;

    protected:
      enum DrainState
      {
        NotDraining,      /* Not draining, possibly running */
        DrainCurrentInst, /* Draining to end of inst/macroop */
        DrainHaltFetch,   /* Halting Fetch after completing current inst */
        DrainAllInsts     /* Discarding all remaining insts */
      };
      struct MemoryThreadInfo
      {
        MemoryThreadInfo(unsigned int insts_committed) : lastCommitWasEndOfMacroop(true),
                                                         inputIndex(0),
                                                         instsBeingCommitted(insts_committed),
                                                         streamSeqNum(InstId::firstStreamSeqNum),
                                                         lastPredictionSeqNum(InstId::firstPredictionSeqNum),
                                                         drainState(NotDraining) {}
        MemoryThreadInfo(MemoryThreadInfo &other) : lastCommitWasEndOfMacroop(other.lastCommitWasEndOfMacroop),
                                                    inputIndex(other.inputIndex),
                                                    instsBeingCommitted(other.instsBeingCommitted),
                                                    streamSeqNum(other.streamSeqNum),
                                                    lastPredictionSeqNum(other.lastPredictionSeqNum),
                                                    drainState(other.drainState) {}
        Queue<QueuedInst, ReportTraitsAdaptor<QueuedInst>> *inFlightInsts;
        Queue<QueuedInst, ReportTraitsAdaptor<QueuedInst>> *inMemInsts;
        MinorDynInstPtr inFlightInst;

        bool lastCommitWasEndOfMacroop;
        unsigned int inputIndex;
        ForwardInstData instsBeingCommitted;
        InstSeqNum streamSeqNum;
        InstSeqNum lastPredictionSeqNum;
        DrainState drainState;
      };

      std::vector<MemoryThreadInfo>
          memoryInfo;

      std::vector<Scoreboard> &scoreboard;

      ThreadID interruptPriority;
      ThreadID issuePriority;
      ThreadID commitPriority;

    protected:
      friend std::ostream &operator<<(std::ostream &os, DrainState &state);
      const ForwardInstData *getInput(ThreadID tid);

      void popInput(ThreadID tid);

      void tryToBranch(MinorDynInstPtr inst, Fault fault, BranchData &branch);

      void updateBranchData(ThreadID tid, BranchData::Reason reason,
                            MinorDynInstPtr inst, const PCStateBase &target, BranchData &branch);

      void handleMemResponse(MinorDynInstPtr inst, LSQ::LSQRequestPtr response, BranchData &branch, Fault &fault);

      bool executeMemRefInst(MinorDynInstPtr inst, BranchData &branch, bool &passed_predicate, Fault &fault);

      bool tryPCEvents(ThreadID thread_id);
      bool isInbetweenInsts(ThreadID thread_id) const;

      void changeStream(const BranchData &branch);
      void updateExpectedSeqNums(const BranchData &branch);
      void processBranchesFromLaterStages(const BranchData &branch_in);

      bool commitInst(MinorDynInstPtr inst, ForwardInstData &insts_out, unsigned int *output_index, bool early_memory_issue,
                      BranchData &branch, Fault &fault, bool &issued_mem_ref);

      void sendOutput(ThreadID thread_id, ForwardInstData &insts_out, unsigned int *output_index,
                      bool only_commit_microops, bool discard,
                      BranchData &branch);

      void packIntoOutput(
          MinorDynInstPtr output_inst,
          ForwardInstData &insts_out,
          unsigned int *output_index)
      {
        /* Correctly size the output before writing */
        if (*output_index == 0)
        {
          insts_out.resize(outputWidth);
        }

        /* Push into output */
        insts_out.insts[*output_index] = output_inst;

        (*output_index) += 1;
      }

      /** Set the drain state (with useful debugging messages) */
      void setDrainState(ThreadID thread_id, DrainState state);

      /** Use the current threading policy to determine the next thread to
       *  decode from. */
      ThreadID getCommittingThread();
      ThreadID getIssuingThread();

    public:
      Memory(const std::string &name_,
             MinorCPU &cpu_,
             const BaseMinorCPUParams &params,
             Latch<ForwardInstData>::Output inp_,
             Latch<ForwardInstData>::Input out_,
             Latch<BranchData>::Input branch_out_,
             Latch<BranchData>::Output branch_in_,
             std::vector<Scoreboard> &scoreboard_,
             std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer);
      ~Memory();

    public:
      MinorCPU::MinorCPUPort &getDcachePort();
      LSQ &getLSQ() { return cpu.getLSQ(); };

      void evaluate();
      void minorTrace() const;
      bool isDrained();
      unsigned int drain();
      void drainResume();

    private:
      bool checkForIssuableInsts(std::vector<MinorDynInstPtr> &next_issuable_insts);
      bool headInstMightCommit(LSQ &lsq);
      bool attemptIssue(ThreadID tid);
      void attemptCommit(ThreadID commit_tid, ForwardInstData &insts_out, unsigned int *output_index, BranchData &branch, bool interrupted);

      void tryToHandleMemResponses(MemoryThreadInfo &mem_info, bool discard_inst,
                                   bool &committed_inst,
                                   bool &completed_mem_ref,
                                   bool &completed_inst,
                                   MinorDynInstPtr inst,
                                   LSQ::LSQRequestPtr mem_response,
                                   BranchData &branch,
                                   Fault &fault);

      void finalizeCompletedInstruction(ThreadID thread_id, const MinorDynInstPtr inst, MemoryThreadInfo &mem_info, const Fault &fault, bool issued_mem_ref, bool committed_inst);
      void startMemRefExecution(MinorDynInstPtr inst, BranchData &branch, Fault &fault, gem5::ThreadContext *thread, bool &completed_inst, bool &completed_mem_issue);
      void actuallyExecuteInst(ThreadID thread_id, MinorDynInstPtr inst, Fault &fault, gem5::ThreadContext *thread, bool &committed);
      void checkSuspension(ThreadID thread_id, MinorDynInstPtr inst, Fault &fault, gem5::ThreadContext *thread, BranchData &branch);
    };
  } // namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_EXECUTE_HH__ */
