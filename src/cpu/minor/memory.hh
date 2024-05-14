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

    /** Input port carrying instructions from Decode */
    Latch<ForwardInstData>::Output inp;

    /** Input port carrying stream changes to Fetch1 */
    Latch<BranchData>::Input out;

    Latch<ForwardInstData>::Input out_insts;

    /** Pointer back to the containing CPU */
    MinorCPU &cpu;

    /** Number of instructions that can be issued per cycle */
    unsigned int issueLimit;

    /** Number of memory ops that can be issued per cycle */
    unsigned int memoryIssueLimit;

    /** Number of instructions that can be committed per cycle */
    unsigned int commitLimit;

    /** Number of memory instructions that can be committed per cycle */
    unsigned int memoryCommitLimit;

    std::vector<InputBuffer<ForwardInstData>> &nextStageReserve;

    /** Width of output of this stage/input of next in instructions */
    unsigned int outputWidth;

    /** If true, more than one input line can be processed each cycle if
     *  there is room to execute more instructions than taken from the first
     *  line */
    bool processMoreThanOneInput;

    /** Descriptions of the functional units we want to generate */
    MinorFUPool &fuDescriptions;

    /** Number of functional units to produce */
    unsigned int numFuncUnits;

    /** Longest latency of any FU, useful for setting up the activity
     *  recoder */
    Cycles longestFuLatency;

    /** Modify instruction trace times on commit */
    bool setTraceTimeOnCommit;

    /** Modify instruction trace times on issue */
    bool setTraceTimeOnIssue;

    /** Allow mem refs to leave their FUs before reaching the head
     *  of the in flight insts queue if their dependencies are met */
    bool allowEarlyMemIssue;

    /** The FU index of the non-existent costless FU for instructions
     *  which pass the MinorDynInst::isNoCostInst test */
    unsigned int noCostFUIndex;

    /** Dcache port to pass on to the CPU.  Memory owns this */
    //LSQ& lsq;


  public: 
  /* Public for Pipeline to be able to pass it to Decode */
    std::vector<InputBuffer<ForwardInstData>> inputBuffer;

    /** Scoreboard of instruction dependencies */
    std::vector<Scoreboard> scoreboard;

    /** The execution functional units */
    std::vector<FUPipeline *> funcUnits;

  protected:
    /** Stage cycle-by-cycle state */

    /** State that drain passes through (in order).  On a drain request,
     *  Memory transitions into either DrainCurrentInst (if between
     *  microops) or DrainHaltFetch.
     *
     *  Note that Memory doesn't actually have *  a 'Drained' state, only
     *  an indication that it's currently draining and isDrained that can't
     *  tell if there are insts still in the pipeline leading up to
     *  Memory */
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

        /** In-order instructions either in FUs or the LSQ */
        Queue<QueuedInst, ReportTraitsAdaptor<QueuedInst> > *inFlightInsts;

        /** Memory ref instructions still in the FUs */
        Queue<QueuedInst, ReportTraitsAdaptor<QueuedInst> > *inFUMemInsts;

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
    ThreadID issuePriority;
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

    /** Handle extracting mem ref responses from the memory queues and
     *  completing the associated instructions.
     *  Fault is an output and will contain any fault caused (and already
     *  invoked by the function)
     *  Sets branch to any branch generated by the instruction. */
    void handleMemResponse(MinorDynInstPtr inst,
        LSQ::LSQRequestPtr response, BranchData &branch,
        Fault &fault);

    /** Memory a memory reference instruction.  This calls initiateAcc on
     *  the instruction which will then call writeMem or readMem to issue a
     *  memory access to the LSQ.
     *  Returns true if the instruction was executed rather than stalled
     *  because of a lack of LSQ resources and false otherwise.
     *  branch is set to any branch raised by the instruction.
     *  failed_predicate is set to false if the instruction passed its
     *  predicate and so will access memory or true if the instruction
     *  *failed* its predicate and is now complete.
     *  fault is set if any non-NoFault fault is raised.
     *  Any faults raised are actually invoke-d by this function. */
    bool executeMemRefInst(MinorDynInstPtr inst, BranchData &branch,
        bool &failed_predicate, Fault &fault);

    /** Has an interrupt been raised */
    bool isInterrupted(ThreadID thread_id) const;

    /** Are we between instructions?  Can we be interrupted? */
    bool isInbetweenInsts(ThreadID thread_id) const;

    /** Act on an interrupt.  Returns true if an interrupt was actually
     *  signalled and invoked */
    bool takeInterrupt(ThreadID thread_id, BranchData &branch);

    /** Issues a non-cost instruction to the specified thread's
     * noCost functional unit */
    void issueNoCostInst(ThreadID thread_id, MinorDynInstPtr inst);

    /** Inserts an instruction into a functional unit pipeline for
      * execution, and marks its destinations as busy in the scoreboard.*/
    void insertIntoFU(ThreadID thread_id, MinorDynInstPtr inst,
        MinorFUTiming *timing, int fu_index);

    /** Determines if an instruction can be issued to a given functional
     *  unit pipeline. Returns true if the functional unit is capable of
     *  executing the instruction's operation class, is not currently
     *  busy, not stalled, and can insert the instruction without
     *  delaying other queued instructions. Returns false otherwise. */
    bool canFUIssueInst(MinorDynInstPtr inst, FUPipeline* fu, int fu_index);

    /** Attempts to issue an instruction to a functional unit pipeline.
     * Returns true if the instruction is issued correctly. Returns false
     * otherwise */
    bool tryIssueInstruction(ThreadID thread_id, const MinorDynInstPtr &inst,
        unsigned int &fu_index, bool &discarded, bool &issued_mem_ref);



    /** Try and issue instructions from the inputBuffer */
    unsigned int issue(ThreadID thread_id);

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
    bool commitInst(MinorDynInstPtr inst, ForwardInstData &insts_out, unsigned int *output_index, bool early_memory_issue,
        BranchData &branch, Fault &fault);

    /** Try and commit instructions from the ends of the functional unit
     *  pipelines.
     *  If only_commit_microops is true then only commit upto the
     *  end of the currect full instruction.
     *  If discard is true then discard all instructions rather than
     *  committing.
     *  branch is set to any branch raised during commit. */
    void commit(ThreadID thread_id, bool only_commit_microops, bool discard,
        BranchData &branch);

    void sendOutput(ThreadID thread_id, ForwardInstData &insts_out, unsigned int *output_index,
                    bool only_commit_microops, bool discard,
        BranchData &branch);

    void packIntoOutput(
    MinorDynInstPtr output_inst,
    ForwardInstData &insts_out,
    unsigned int *output_index)
{
    /* Correctly size the output before writing */
    if (*output_index == 0) {
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
        Latch<BranchData>::Input branch_out_,
        std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer,
        Latch<ForwardInstData>::Input  out_);

    ~Memory();

  public:

    /** Returns the DcachePort owned by this Memory to pass upwards */
    MinorCPU::MinorCPUPort &getDcachePort();

    /** To allow ExecContext to find the LSQ */
    LSQ &getLSQ() { return cpu.getLSQ(); }

    /** Does the given instruction have the right stream sequence number
     *  to be committed? */
    bool instIsRightStream(MinorDynInstPtr inst);

    /** Returns true if the given instruction is at the head of the
     *  inFlightInsts instruction queue */
    bool instIsHeadInst(MinorDynInstPtr inst);

    /** Pass on input/buffer data to the output if you can */
    void evaluate();

    void minorTrace() const;

    /** After thread suspension, has Memory been drained of in-flight
     *  instructions and memory accesses. */
    bool isDrained();

    /** Like the drain interface on SimObject */
    unsigned int drain();
    void drainResume();

    std::vector<Scoreboard>& getScoreboard() { return scoreboard; }
  /** Refactor methods */
  private:
    // Memory::evaluate();
    bool checkForIssuableInsts(std::vector<MinorDynInstPtr> &next_issuable_insts);
    void advanceFunctionalUnits(std::vector<MinorDynInstPtr> &next_issuable_insts, bool &becoming_stalled, bool &can_issue_next);
    bool headInstMightCommit(LSQ &lsq);
    void attemptCommit(ThreadID commit_tid, ForwardInstData &insts_out, unsigned int *output_index, BranchData &branch, bool interrupted);

   // Memory::commit()
   /** Tries to commit memory responses (or discard them) */
    void tryToHandleMemResponses(MemoryThreadInfo &ex_info, bool discard_inst,
      bool &committed_inst,
      bool &completed_mem_ref,
      bool &completed_inst,
      MinorDynInstPtr inst,
      LSQ::LSQRequestPtr mem_response,
      BranchData &branch,
      Fault &fault);

    /** Checks if an early memory issue is possible. If it is, sets completed_inst to true
     * and sets inst to the instruction corresponding to the memory issue, so that it may be
     * later committed.
     */
    void checkIfEarlyMemIssuePossible(MemoryThreadInfo &ex_info, MinorDynInstPtr* inst, bool &try_to_commit, bool &early_memory_issue, bool &completed_inst, InstSeqNum head_exec_seq_num);

    /** Check if the instruction has reached the end of a FU and can be committed.
     * If it is, set try_to_commit and completed_inst to true
     */
    void checkIfCommitFromFUsPossible(const MinorDynInstPtr inst, bool &completed_inst, bool &try_to_commit, InstSeqNum head_exec_seq_num);
    /** Check if the instruction needs further delay to commit. */
    void checkExtraCommitDelay(ThreadID thread_id, MinorDynInstPtr const inst);

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
    void finalizeCompletedInstruction(ThreadID thread_id, const MinorDynInstPtr inst, MemoryThreadInfo &ex_info, const Fault &fault, bool issued_mem_ref, bool committed_inst);

    // Memory::commitInst()
    /** Calculate the effective address and issue the access to memory */
    void startMemRefExecution(MinorDynInstPtr inst, BranchData &branch, Fault &fault, gem5::ThreadContext *thread, bool early_memory_issue, bool &completed_inst, bool &completed_mem_issue);
    /** Perform the actual execution of the instruction. If a fault happened, invoke it. */
    void actuallyExecuteInst(ThreadID thread_id, MinorDynInstPtr inst, Fault &fault, gem5::ThreadContext *thread, bool &committed);
    /** Check if the thread has been suspended. */
    void checkSuspension(ThreadID thread_id, MinorDynInstPtr inst, gem5::ThreadContext *thread, BranchData &branch);
};

} // namespace minor
} // namespace gem5

#endif /* __CPU_MINOR_EXECUTE_HH__ */
