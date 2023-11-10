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

#ifndef __CPU_MINOR_WRITEBACK_HH__
#define __CPU_MINOR_WRITEBACK_HH__

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

/** Execute stage.  Everything apart from fetching and decoding instructions.
 *  The LSQ lives here too. */
class Memory : public Named {
protected:

    /** Input port carrying stream changes to Fetch1 */
    Latch<BranchData>::Input branch_out;
    
    /** Input port to send instructions to Memory */
    Latch<ForwardInstData>::Input out;
    
    /** Output port carrying instructions from Execute */
    Latch<ForwardInstData>::Output inp;

    /** Pointer back to the containing CPU */
    MinorCPU &cpu;

    std::vector<InputBuffer<ForwardInstData>> &nextStageReserve;

    /** Number of memory instructions that can be committed per cycle */
    unsigned int memoryCommitLimit;

    /** If true, more than one input line can be processed each cycle if
             *  there is room to execute more instructions than taken from the first
             *  line */
    bool processMoreThanOneInput;

    /** Modify instruction trace times on commit */
    bool setTraceTimeOnCommit;

public:
    /* Public for Pipeline to be able to pass it to Decode */
    std::vector<InputBuffer<ForwardInstData>> inputBuffer;

protected:
    /** Stage cycle-by-cycle state */

    /** State that drain passes through (in order).  On a drain request,
     *  Execute transitions into either DrainCurrentInst (if between
     *  microops) or DrainHaltFetch.
     *
     *  Note that Execute doesn't actually have *  a 'Drained' state, only
     *  an indication that it's currently draining and isDrained that can't
     *  tell if there are insts still in the pipeline leading up to
     *  Execute */
    enum DrainState
    {
        NotDraining, /* Not draining, possibly running */
        DrainCurrentInst, /* Draining to end of inst/macroop */
        DrainHaltFetch, /* Halting Fetch after completing current inst */
        DrainAllInsts /* Discarding all remaining insts */
    };

    struct MemoryThreadInfo
    {
        /** Constructor */
        MemoryThreadInfo(unsigned int insts_committed) :
                inputIndex(0),
                lastCommitWasEndOfMacroop(true),
                instsBeingCommitted(insts_committed),
                streamSeqNum(InstId::firstStreamSeqNum),
                lastPredictionSeqNum(InstId::firstPredictionSeqNum),
                drainState(NotDraining)
        { }

        MemoryThreadInfo(const MemoryThreadInfo& other) :
                inputIndex(other.inputIndex),
                lastCommitWasEndOfMacroop(other.lastCommitWasEndOfMacroop),
                instsBeingCommitted(other.instsBeingCommitted),
                streamSeqNum(other.streamSeqNum),
                lastPredictionSeqNum(other.lastPredictionSeqNum),
                drainState(other.drainState)
        { }

        /** Index that we've completed upto in getInput data.  We can say we're
         *  popInput when this equals getInput()->width() */
        unsigned int inputIndex;

        /** The last commit was the end of a full instruction so an interrupt
         *  can safely happen */
        bool lastCommitWasEndOfMacroop;

        /** Structure for reporting insts currently being processed/retired
         *  for MinorTrace */
        ForwardInstData instsBeingCommitted;

        /** Source of sequence number for instuction streams.  Increment this and
         *  pass to fetch whenever an instruction stream needs to be changed.
         *  For any more complicated behaviour (e.g. speculation) there'll need
         *  to be another plan. */
        InstSeqNum streamSeqNum;

        /** A prediction number for use where one isn't available from an
         *  instruction.  This is harvested from committed instructions.
         *  This isn't really needed as the streamSeqNum will change on
         *  a branch, but it minimises disruption in stream identification */
        InstSeqNum lastPredictionSeqNum;

        /** State progression for draining NotDraining -> ... -> DrainAllInsts */
        DrainState drainState;
    };

    std::vector<MemoryThreadInfo> memoryInfo;

    ThreadID interruptPriority;
    ThreadID commitPriority;

protected:
    friend std::ostream &operator <<(std::ostream &os, DrainState state);

    /** Get a piece of data to work on from the inputBuffer, or 0 if there
     *  is no data. */
    const ForwardInstData *getInput(ThreadID tid);

    /** Pop an element off the input buffer, if there are any */
    void popInput(ThreadID tid);

    /** Generate Branch data based (into branch) on an observed (or not)
     *  change in PC while executing an instruction.
     *  Also handles branch prediction information within the inst. */
    void tryToBranch(MinorDynInstPtr inst, Fault fault, BranchData &branch);

    /** Actually create a branch to communicate to Fetch1/Fetch2 and,
     *  if that is a stream-changing branch update the streamSeqNum */
    void updateBranchData(ThreadID tid, BranchData::Reason reason,
                          MinorDynInstPtr inst, const PCStateBase &target, BranchData &branch);

    /** Has an interrupt been raised */
    bool isInterrupted(ThreadID thread_id) const;

    /** Are we between instructions?  Can we be interrupted? */
    bool isInbetweenInsts(ThreadID thread_id) const;

    /** Act on an interrupt.  Returns true if an interrupt was actually
     *  signalled and invoked */
    bool takeInterrupt(ThreadID thread_id, BranchData &branch);

    /** Try to act on PC-related events.  Returns true if any were
     *  executed */
    bool tryPCEvents(ThreadID thread_id);

    /** Do the stats handling and instruction count and PC event events
     *  related to the new instruction/op counts */
    void doInstCommitAccounting(MinorDynInstPtr inst);

    /** Check all threads for possible interrupts. If interrupt is taken,
     *  returns the tid of the thread.  interrupted is set if any thread
     *  has an interrupt, irrespective of if it is taken */
    ThreadID checkInterrupts(BranchData& branch, bool& interrupted);

    /** Checks if a specific thread has an interrupt.  No action is taken.
     *  this is used for determining if a thread should only commit microops */
    bool hasInterrupt(ThreadID thread_id);

    /** Commit a single instruction.  Returns true if the instruction being
     *  examined was completed (fully executed, discarded, or initiated a
     *  memory access), false if there is still some processing to do.
     *  fu_index is the index of the functional unit this instruction is
     *  being executed in into for funcUnits
     *  If early_memory_issue is true then this is an early execution
     *  of a mem ref and so faults will not be processed.
     *  If the return value is true:
     *      fault is set if a fault happened,
     *      branch is set to indicate any branch that occurs
     *      committed is set to true if this instruction is committed
     *          (and so needs to be traced and accounted for)
     *      completed_mem_issue is set if the instruction was a
     *          memory access that was issued */
    bool commitInst(MinorDynInstPtr inst,
                    BranchData &branch, Fault &fault, bool &committed
    );

    /** Try and commit instructions from the ends of the functional unit
     *  pipelines.
     *  If only_commit_microops is true then only commit upto the
     *  end of the currect full instruction.
     *  If discard is true then discard all instructions rather than
     *  committing.
     *  branch is set to any branch raised during commit. */
    void commit(ThreadID thread_id, bool only_commit_microops, bool discard,
                BranchData &branch);

    /** Set the drain state (with useful debugging messages) */
    void setDrainState(ThreadID thread_id, DrainState state);

    /** Use the current threading policy to determine the next thread to
     *  decode from. */
    ThreadID getCommittingThread();
public:
    Memory(const std::string &name_,
              MinorCPU &cpu_,
              const BaseMinorCPUParams &params,
              Latch<ForwardInstData>::Output inp_,
              std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer,
              Latch<BranchData>::Input out_);

    ~Memory();

public:
    /** Pass on input/buffer data to the output if you can */
    void evaluate();

    void minorTrace() const;

    /** After thread suspension, has Execute been drained of in-flight
     *  instructions and memory accesses. */
    bool isDrained();

    /** Like the drain interface on SimObject */
    unsigned int drain();
    void drainResume();
    /** Refactor methods */
private:
    /** Actually try to commit the instruction. Commit fails if there are incomplete memory barriers
     * there is extra commit delay to account for (see checkExtraCommitDelay) or commitInst returns false.
     * If the instruction has been completed, unstall its functional unit.
    */
    bool tryCommit(ThreadID thread_id,
                   const MinorDynInstPtr inst,
                   BranchData &branch, Fault &fault,
                   Cycles now,
                   bool discard_inst,
                   bool early_memory_issue,
                   bool committed_inst,
                   bool issued_mem_ref
    );

    /** Update commit statistics */
    void doCommitAccounting(MinorDynInstPtr const inst, MemoryThreadInfo &ex_info, unsigned int &num_insts_committed, unsigned int &num_mem_refs_committed, bool completed_mem_ref);
    /** Final commit operations. Pop the instruction from the queue of instructions in memory FUs (if it was a memory instruction),
     * pop the instruction from the queue of in-flight instructions (if it wasn't a memory instruction), complete the memory
     * barrier (if it was a memory barrier instruction) and update the scoreboard.
    */
    void finalizeCompletedInstruction(ThreadID thread_id, const MinorDynInstPtr inst, MemoryThreadInfo &ex_info, const Fault &fault, bool committed_inst);

    /** Perform the actual execution of the instruction. If a fault happened, invoke it. */
    void actuallyExecuteInst(ThreadID thread_id, MinorDynInstPtr inst, Fault &fault, gem5::ThreadContext *thread, bool &committed);
    /** Check if the thread has been suspended. */
    void checkSuspension(ThreadID thread_id, MinorDynInstPtr inst, gem5::ThreadContext *thread, BranchData &branch);
};

} // namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_WRITEBACK_HH__ */
