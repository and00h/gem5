#include "cpu/minor/memory.hh"

#include "cpu/minor/cpu.hh"
#include "cpu/minor/exec_context.hh"
#include "cpu/minor/fetch1.hh"
#include "cpu/minor/lsq.hh"
#include "cpu/op_class.hh"
#include "debug/MinorMem.hh"
#include "debug/MinorMemory.hh"
#include "debug/Branch.hh"
#include "debug/Activity.hh"

namespace gem5
{
    GEM5_DEPRECATED_NAMESPACE(Minor, minor);

    namespace minor
    {
        Memory::Memory(const std::string &name_,
                       MinorCPU &cpu_,
                       const BaseMinorCPUParams &params,
                       Latch<ForwardInstData>::Output inp_,
                       Latch<ForwardInstData>::Input out_,
                       Latch<BranchData>::Input branch_out_,
                       Latch<BranchData>::Output branch_in_,
                       std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer)
            : Named(name_),
              inp(inp_),
              out(out_),
              branch_out(branch_out_),
              branch_in(branch_in_),
              cpu(cpu_),
              issueLimit(1),
              memoryIssueLimit(1),
              commitLimit(1),
              memoryCommitLimit(1),
              nextStageReserve(next_stage_input_buffer),
              outputWidth(1),
              processMoreThanOneInput(false),
              setTraceTimeOnCommit(params.executeSetTraceTimeOnCommit)
        {
            DPRINTF(MinorMem, "Created memory stage.\n");
            if (commitLimit < 1)
            {
                fatal("%s: Commit limit must be >= 1 (%d)\n", name_, commitLimit);
            }

            if (issueLimit < 1)
            {
                fatal("%s: Issue limit must be >= 1 (%d)\n", name_, issueLimit);
            }

            if (memoryIssueLimit < 1)
            {
                fatal("%s: Memory issue limit must be >= 1 (%d)\n", name_,
                      memoryIssueLimit);
            }

            if (memoryCommitLimit > commitLimit)
            {
                fatal("%s: Memory commit limit (%d) must be <= commit limit (%d)\n",
                      name_, memoryCommitLimit, commitLimit);
            }

            if (params.executeInputBufferSize < 1)
            {
                fatal("%s: Input buffer size must be >= 1 (%d)\n", name_,
                      params.executeInputBufferSize);
            }

            for (ThreadID tid = 0; tid < params.numThreads; tid++)
            {
                std::string tid_str = std::to_string(tid);
                inputBuffer.push_back(
                    InputBuffer<ForwardInstData>(name_ + ".inputBuffer" + tid_str, "insts",
                                                 params.executeInputBufferSize));
                memoryInfo[tid].inFlightInsts = new Queue<QueuedInst,
                                                          ReportTraitsAdaptor<QueuedInst>>(
                    name_ + ".inFlightInsts" + tid_str, "insts", params.executeInputBufferSize);
                memoryInfo[tid].inMemInsts = new Queue<QueuedInst,
                                                       ReportTraitsAdaptor<QueuedInst>>(
                    name_ + ".inMemInsts" + tid_str, "insts", params.executeInputBufferSize);
            }
        }
        const ForwardInstData *
        Memory::getInput(ThreadID tid)
        {
            if (inputBuffer[tid].empty())
            {
                return NULL;
            }
            else
            {
                const ForwardInstData &head = inputBuffer[tid].front();
                return (head.isBubble() ? NULL : &(inputBuffer[tid].front()));
            }
        }

        void
        Memory::popInput(ThreadID tid)
        {
            if (!inputBuffer[tid].empty())
            {
                inputBuffer[tid].pop();
            }

            memoryInfo[tid].inputIndex = 0;
        }

        void
        Memory::tryToBranch(MinorDynInstPtr inst, Fault fault, BranchData &branch)
        {
            ThreadContext *thread = cpu.getContext(inst->id.threadId);
            const std::unique_ptr<PCStateBase> pc_before(inst->pc->clone());
            std::unique_ptr<PCStateBase> target(thread->pcState().clone());

            bool force_branch = thread->status() != ThreadContext::Suspended &&
                                !inst->isFault() &&
                                inst->isLastOpInInst() &&
                                (inst->staticInst->isSerializeAfter() ||
                                 inst->staticInst->isSquashAfter());

            DPRINTF(Branch, "tryToBranch before: %s after: %s%s\n",
                    *pc_before, *target, (force_branch ? " (forcing)" : ""));

            bool must_branch = *pc_before != *target ||
                               fault != NoFault ||
                               force_branch;

            BranchData::Reason reason = BranchData::NoBranch;
            if (fault == NoFault)
            {
                inst->staticInst->advancePC(*target);
                thread->pcState(*target);

                DPRINTF(Branch, "Advancing current PC from: %s to: %s\n",
                        *pc_before, *target);
            }

            if (inst->predictedTaken && !force_branch)
            {
                if (!must_branch)
                {
                    DPRINTF(Branch, "Predicted a branch from 0x%x to 0x%x but none happened inst: %s\n",
                            inst->pc->instAddr(), inst->predictedTarget->instAddr(), *inst);
                    reason = BranchData::BadlyPredictedBranch;
                }
                else if (*inst->predictedTarget == *target)
                {
                    DPRINTF(Branch, "Predicted a branch from 0x%x to 0x%x correctly"
                                    " inst: %s\n",
                            inst->pc->instAddr(), inst->predictedTarget->instAddr(),
                            *inst);

                    reason = BranchData::CorrectlyPredictedBranch;
                }
                else
                {
                    DPRINTF(Branch, "Predicted a branch from 0x%x to 0x%x"
                                    " but got the wrong target (actual: 0x%x) inst: %s\n",
                            inst->pc->instAddr(), inst->predictedTarget->instAddr(),
                            target->instAddr(), *inst);

                    reason = BranchData::BadlyPredictedBranchTarget;
                }
            }
            else if (must_branch)
            {
                DPRINTF(Branch, "Unpredicted branch from 0x%x to 0x%x inst: %s\n",
                        inst->pc->instAddr(), target->instAddr(), *inst);

                reason = BranchData::UnpredictedBranch;
            }
            else
            {
                reason = BranchData::NoBranch;
            }

            updateBranchData(inst->id.threadId, reason, inst, *target, branch);
        }

        void
        Memory::updateBranchData(ThreadID tid, BranchData::Reason reason, MinorDynInstPtr inst, const PCStateBase &target, BranchData &branch)
        {
            if (reason != BranchData::NoBranch)
            {
                if (BranchData::isStreamChange(reason))
                    memoryInfo[tid].streamSeqNum++;
                branch = BranchData(reason, tid, memoryInfo[tid].streamSeqNum,
                                    (inst->isBubble() ? memoryInfo[tid].lastPredictionSeqNum : inst->id.predictionSeqNum),
                                    target, inst);
            }
        }

        void
        Memory::handleMemResponse(MinorDynInstPtr inst, LSQ::LSQRequestPtr response, BranchData &branch, Fault &fault)
        {
            LSQ &lsq = cpu.getLSQ();
            ThreadID thread_id = inst->id.threadId;
            ThreadContext *thread = cpu.getContext(thread_id);

            ExecContext context(cpu, *cpu.threads[thread_id], inst);
            PacketPtr packet = response->packet;

            bool is_load = inst->staticInst->isLoad();
            bool is_store = inst->staticInst->isStore();
            bool is_atomic = inst->staticInst->isAtomic();
            bool is_prefetch = inst->staticInst->isDataPrefetch();

            bool use_context_predicate = true;

            if (inst->translationFault != NoFault)
            {
                DPRINTF(MinorMem, "Completing fault from DTLB access: %s\n",
                        inst->translationFault->name());
                if (is_prefetch)
                {
                    DPRINTF(MinorMem, "Not taking fault on prefetch: %s\n", inst->translationFault->name());
                }
                else
                {
                    fault = inst->translationFault;
                    fault->invoke(thread, inst->staticInst);
                }
            }
            else if (!packet)
            {
                DPRINTF(MinorMem, "Completing failed request inst: %s\n", *inst);
                use_context_predicate = false;
                if (!context.readMemAccPredicate())
                    inst->staticInst->completeAcc(nullptr, &context, inst->traceData);
            }
            else if (packet->isError())
            {
                DPRINTF(MinorMem, "Trying to commit error response: %s\n", *inst);

                fatal("Received error response packet for inst: %s\n", *inst);
            }
            else if (is_store || is_load || is_prefetch || is_atomic)
            {
                assert(packet);

                DPRINTF(MinorMem, "Memory response inst: %s addr: 0x%x size: %d\n",
                        *inst, packet->getAddr(), packet->getSize());
                if (is_load && packet->getSize() > 0)
                {
                    DPRINTF(MinorMem, "Memory data[0]: 0x%x\n",
                            static_cast<unsigned int>(packet->getConstPtr<uint8_t>()[0]));
                }

                fault = inst->staticInst->completeAcc(packet, &context, inst->traceData);
                inst->executed = true;
                context.writeback(inst->staticInst);

                if (fault != NoFault)
                {
                    DPRINTF(MinorMem, "Fault in memory completeAcc: %s\n",
                            fault->name());
                    fault->invoke(thread, inst->staticInst);
                }
                else
                {
                    if (response->needsToBeSentToStoreBuffer())
                        lsq.sendStoreToStoreBuffer(response);
                }
            }
            else
            {
                fatal("There should only ever be reads, writes or faults at this point\n");
            }

            lsq.popResponse(response);

            if (inst->traceData)
            {
                inst->traceData->setPredicate((use_context_predicate ? context.readPredicate() : false));
            }

            // doInstCommitAccounting(inst);
            tryToBranch(inst, fault, branch);
        }

        bool
        Memory::executeMemRefInst(MinorDynInstPtr inst, BranchData &branch, bool &passed_predicate, Fault &fault)
        {
            bool issued = false;
            passed_predicate = false;

            if (!cpu.getLSQ().canRequest())
            {
                issued = false;
            }
            else
            {
                ThreadContext *thread = cpu.getContext(inst->id.threadId);
                std::unique_ptr<PCStateBase> old_pc(thread->pcState().clone());

                ExecContext context(cpu, *cpu.threads[inst->id.threadId], inst);

                DPRINTF(MinorMem, "Initiating memRef inst: %d\n", *inst);

                Fault init_fault = inst->staticInst->initiateAcc(&context, inst->traceData);

                if (inst->inLSQ)
                {
                    if (init_fault != NoFault)
                    {
                        assert(inst->translationFault != NoFault);
                        init_fault = NoFault;
                    }
                    else
                    {
                        inst->translationFault = NoFault;
                    }
                }

                if (init_fault != NoFault)
                {
                    DPRINTF(MinorMem, "Fault on memory inst: %s initiateAcc: %s\n", *inst, init_fault->name());
                    fault = init_fault;
                }
                else
                {
                    if (!context.readMemAccPredicate())
                    {
                        DPRINTF(MinorMem, "No memory access for inst: %s\n", *inst);
                        assert(context.readPredicate());
                    }
                    passed_predicate = context.readPredicate();

                    if (inst->traceData)
                        inst->traceData->setPredicate(passed_predicate);

                    if (!inst->inLSQ)
                    {
                        cpu.getLSQ().pushFailedRequest(inst);
                        inst->inLSQ = true;
                    }
                }

                thread->pcState(*old_pc);
                issued = true;
            }

            return issued;
        }

        void
        Memory::tryToHandleMemResponses(
            MemoryThreadInfo &ex_info,
            bool discard_inst,
            bool &committed_inst,
            bool &completed_mem_ref,
            bool &completed_inst,
            MinorDynInstPtr inst,
            LSQ::LSQRequestPtr mem_response,
            BranchData &branch,
            Fault &fault)
        {
            DPRINTF(MinorMemory, "Trying to commit mem response: %s\n", *inst);

            /* Complete or discard the response */
            if (discard_inst)
            {
                DPRINTF(MinorMemory, "Discarding mem inst: %s as its"
                                     " stream state was unexpected, expected: %d\n",
                        *inst, ex_info.streamSeqNum);

                cpu.getLSQ().popResponse(mem_response);
            }
            else
            {
                handleMemResponse(inst, mem_response, branch, fault);
                committed_inst = true;
            }

            completed_mem_ref = true;
            completed_inst = true;
        }

        bool
        Memory::commitInst(MinorDynInstPtr inst, ForwardInstData &insts_out, unsigned int *output_index, bool early_memory_issue,
                           BranchData &branch, Fault &fault, bool &issued_mem_ref)
        {
            ThreadID thread_id = inst->id.threadId;
            ThreadContext *thread = cpu.getContext(thread_id);

            bool completed_inst = true;
            fault = NoFault;

            if (thread->status() == ThreadContext::Suspended
                // && !isInterrupted(thread_id)
            )
            {
                panic("We should never hit the case where we try to commit from a "
                      "suspended thread as the streamSeqNum should not match");
            }
            else if (inst->isFault())
            {
                fault = inst->fault;
                panic("Fault reached memory??????");
                // ExecContext context(cpu, *cpu.threads[thread_id], inst);

                // DPRINTF(MinorExecute, "Fault inst reached Execute: %s\n",
                //         inst->fault->name());

                // fault = inst->fault;
                // inst->fault->invoke(thread, NULL);
                // inst->executed = true;
                // tryToBranch(inst, fault, branch);
            }
            else if (inst->staticInst->isMemRef())
            {
                startMemRefExecution(inst, branch, fault, thread, completed_inst, issued_mem_ref);
            }
            else if (inst->isInst() && inst->staticInst->isFullMemBarrier() &&
                     !cpu.getLSQ().canPushIntoStoreBuffer())
            {
                DPRINTF(MinorExecute, "Can't commit data barrier inst: %s yet as"
                                      " there isn't space in the store buffer\n",
                        *inst);

                completed_inst = false;
            }
            else
            {
                DPRINTF(MinorMemory, "Sending instruction to WB: %s\n", *inst);
                packIntoOutput(inst, insts_out, output_index);
            }

            return completed_inst;
        }

        void Memory::sendOutput(ThreadID thread_id, ForwardInstData &insts_out, unsigned int *output_index, bool only_commit_microops, bool discard, BranchData &branch)
        {
            Fault fault = NoFault;
            Cycles now = cpu.curCycle();
            MemoryThreadInfo &mem_info = memoryInfo[thread_id];
            LSQ &lsq = cpu.getLSQ();

            bool completed_inst = true;
            unsigned int num_insts_committed = 0;
            unsigned int num_mem_refs_committed = 0;

            while (!mem_info.inFlightInsts->empty() &&
                   !branch.isStreamChange() &&
                   fault == NoFault &&
                   completed_inst &&
                   num_insts_committed != commitLimit)
            {

                QueuedInst *head_inflight_inst = &(mem_info.inFlightInsts->front());
                InstSeqNum head_exec_seq_num = head_inflight_inst->inst->id.execSeqNum;
                MinorDynInstPtr inst = head_inflight_inst->inst;

                bool committed_inst = false;
                bool discard_inst = false;
                bool completed_mem_ref = false;
                bool issued_mem_ref = false;

                completed_inst = false;
                LSQ::LSQRequestPtr mem_response = (inst->inLSQ ? lsq.findResponse(inst) : NULL);
                DPRINTF(MinorMem, "Trying to commit canCommitInsts: %d\n", !mem_info.inFlightInsts->empty());
                if (isInbetweenInsts(thread_id) && tryPCEvents(thread_id))
                {
                    ThreadContext *thread = cpu.getContext(thread_id);

                    /* Branch as there was a change in PC */
                    updateBranchData(thread_id, BranchData::UnpredictedBranch,
                                     MinorDynInst::bubble(), thread->pcState(), branch);
                }
                else if (mem_response && num_mem_refs_committed < memoryCommitLimit)
                {
                    discard_inst = inst->id.streamSeqNum != mem_info.streamSeqNum || discard;
                    tryToHandleMemResponses(mem_info, discard_inst, committed_inst, completed_mem_ref, completed_inst, inst, mem_response, branch, fault);
                }
                else if (!mem_info.inFlightInsts->empty())
                {
                    bool try_to_commit = false;
                    discard_inst = inst->id.streamSeqNum != mem_info.streamSeqNum || discard;
                    if (!discard_inst)
                    {
                        if (!inst->isFault() && inst->isMemRef() && lsq.getLastMemBarrier(thread_id) < inst->id.execSeqNum && lsq.getLastMemBarrier(thread_id) != 0)
                        {
                            DPRINTF(MinorMemory, "Not issuing mem ref inst %s due to memory barrier\n", *inst);
                            completed_inst = false;
                        }
                        else
                        {
                            completed_inst = commitInst(inst, insts_out, output_index, false, branch, fault, issued_mem_ref);
                        }
                    }
                    else
                    {
                        completed_inst = true;
                        inst->committed = true;
                    }
                }
                else
                {
                    DPRINTF(MinorMemory, "No inflight insts\n");
                    completed_inst = false;
                }

                assert(!(discard_inst && !completed_inst));

                if (discard_inst)
                {
                    DPRINTF(MinorMem, "Discarding inst: %s as its stream state was unexpected, expected %d\n", *inst, mem_info.streamSeqNum);
                    if (fault == NoFault)
                        cpu.stats.numDiscardedOps++;
                }

                if (completed_inst && inst->isMemRef())
                {
                    if (!mem_info.inMemInsts->empty() && mem_info.inMemInsts->front().inst == inst)
                    {
                        mem_info.inMemInsts->pop();
                    }
                }

                if (issued_mem_ref)
                {
                    inst->fuIndex = 0;
                    inst->inLSQ = true;
                }

                if (completed_inst && !(issued_mem_ref && fault == NoFault))
                {
                    finalizeCompletedInstruction(thread_id, inst, mem_info, fault, issued_mem_ref, committed_inst);
                }
            }
        }

        bool
        Memory::isInbetweenInsts(ThreadID thread_id) const
        {
            return memoryInfo[thread_id].lastCommitWasEndOfMacroop && !cpu.getLSQ().accessesInFlight();
        }

        void
        Memory::changeStream(const BranchData &branch)
        {
            MemoryThreadInfo &thread = memoryInfo[branch.threadId];

            updateExpectedSeqNums(branch);

            /* Start fetching again if we were stopped */
            DPRINTF(MinorMemory, "Changing stream on branch: %s\n", branch);
        }

        void
        Memory::updateExpectedSeqNums(const BranchData &branch)
        {
            MemoryThreadInfo &thread = memoryInfo[branch.threadId];

            DPRINTF(MinorMemory, "Updating streamSeqNum from: %d to %d,",
                    thread.streamSeqNum, branch.newStreamSeqNum);

            /* Change the stream */
            thread.streamSeqNum = branch.newStreamSeqNum;
        }

        void
        Memory::processBranchesFromLaterStages(const BranchData &branch_in)
        {
            bool execute_thread_valid = branch_in.threadId != InvalidThreadID;

            /** Are both branches from later stages valid and for the same thread? */
            if (execute_thread_valid)
            {
                /* Are we changing stream?  Look to the Execute branches first, then
                 * to predicted changes of stream from Fetch2 */
                if (branch_in.isStreamChange())
                {
                    changeStream(branch_in);
                }
            }
        }

        bool Memory::attemptIssue(ThreadID tid)
        {
            const ForwardInstData *insts_in = getInput(tid);
            MemoryThreadInfo &mem_info = memoryInfo[tid];
            if (!insts_in)
            {
                return false;
            }

            bool issued = false;
            while (insts_in && mem_info.inputIndex < insts_in->width() && !issued)
            {
                MinorDynInstPtr inst = insts_in->insts[mem_info.inputIndex];
                Fault fault = inst->fault;
                bool discarded = false;

                if (inst->isBubble())
                {
                    issued = true;
                }
                else if (cpu.getContext(tid)->status() == ThreadContext::Suspended)
                {
                    DPRINTF(MinorMemory, "Discarding inst: %s from suspended thread\n", *inst);
                    issued = true;
                    discarded = true;
                }
                else if (inst->id.streamSeqNum != mem_info.streamSeqNum)
                {
                    DPRINTF(MinorMemory, "Discarding inst: %s as its stream state was unexpected, expected: %d\n", *inst, mem_info.streamSeqNum);
                    issued = true;
                    discarded = true;
                }
                else
                {
                    mem_info.inFlightInst = inst;
                    issued = true;
                }

                if (issued)
                {
                    if (debug::MinorTrace && !inst->isBubble())
                    {
                        inst->minorTraceInst(*this);
                    }
                    mem_info.inputIndex++;
                    DPRINTF(MinorMem, "Stepping to next inst inputIndex: %d\n", mem_info.inputIndex);
                }
                else
                {
                    DPRINTF(MinorMemory, "Not issuing inst: %s\n", *inst);
                }

                if (mem_info.inputIndex == insts_in->width())
                {
                    popInput(tid);
                    insts_in = NULL;
                    DPRINTF(MinorMemory, "Reached end of input\n");
                    if (processMoreThanOneInput)
                    {
                        DPRINTF(MinorMemory, "Wrapping\n");
                        insts_in = getInput(tid);
                    }
                }
            }

            return issued;
        }

        void Memory::attemptCommit(ThreadID commit_tid, ForwardInstData &insts_out, unsigned int *output_index, BranchData &branch, bool interrupted)
        {
            MemoryThreadInfo &mem_info = memoryInfo[commit_tid];
            LSQ &lsq = cpu.getLSQ();
            ThreadContext *thread = cpu.getContext(commit_tid);

            bool completed_inst = true;
            bool committed_inst = false;
            bool completed_mem_ref = false;
            bool issued_mem_ref = false;
            Fault fault = NoFault;
            MinorDynInstPtr inst = mem_info.inFlightInst;

            if (inst && !branch.isStreamChange() && fault == NoFault && !interrupted)
            {
                DPRINTF(MinorMemory, "Trying to pass inst %s\n", *inst);

                bool discard_inst = inst->id.streamSeqNum != mem_info.streamSeqNum;
                if (!discard_inst)
                {
                    completed_inst = commitInst(inst, insts_out, output_index, false, branch, fault, issued_mem_ref);
                }
                else
                {
                    completed_inst = true;
                    inst->committed = true;
                }
            }
            else
            {
                DPRINTF(MinorMemory, "No inflight insts\n");
                completed_inst = false;
            }
            if (issued_mem_ref)
            {
                inst->inLSQ = true;
            }
            if (completed_inst && fault == NoFault)
            {
                finalizeCompletedInstruction(commit_tid, inst, mem_info, fault, issued_mem_ref, committed_inst);
            }
            if (isInbetweenInsts(commit_tid) && tryPCEvents(commit_tid))
            {
                ThreadContext *thread = cpu.getContext(commit_tid);
                updateBranchData(commit_tid, BranchData::UnpredictedBranch,
                                 MinorDynInst::bubble(), thread->pcState(), branch);
            }
        }

        void
        Memory::evaluate()
        {
            if (!inp.outputWire->isBubble())
                inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);

            BranchData &br_in = *branch_in.outputWire;
            BranchData &br_out = *branch_out.inputWire;

            processBranchesFromLaterStages(br_in);

            ForwardInstData &insts_out = *out.inputWire;
            unsigned output_index = 0;

            LSQ &lsq = cpu.getLSQ();
            unsigned int num_mem_insts_issued = 0;
            unsigned int num_mem_insts_committed = 0;

            lsq.step();
            bool interrupted = false;

            ThreadID commit_tid = getCommittingThread();
            ThreadID issue_tid = getIssuingThread();

            if (!br_out.isBubble())
            {
                /* It's important that this is here to carry Fetch1 wakeups to Fetch1
                 *  without overwriting them */
                DPRINTF(MinorMemory, "Memory skipping a cycle to allow old"
                                     " branch to complete\n");
            }
            else
            {
                if (commit_tid != InvalidThreadID)
                {
                    bool issued = attemptIssue(commit_tid);
                    if (issued)
                    {
                        attemptCommit(commit_tid, insts_out, &output_index, br_out, interrupted);
                    }
                }
                // TODO: Memory issue
            }

            bool need_to_tick = false;
            for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
            {
                need_to_tick = need_to_tick || getInput(tid);
            }

            if (!need_to_tick)
            {
                DPRINTF(Activity, "The next cycle might be skippable as there are no"
                                  " instructions to commit\n");
            }

            /* Wake up if we need to tick again */
            if (need_to_tick)
                cpu.wakeupOnEvent(Pipeline::WritebackStageId);

            /* Make sure the input (if any left) is pushed */
            if (!inp.outputWire->isBubble())
                inputBuffer[inp.outputWire->threadId].pushTail();
        }

        inline ThreadID
        Memory::getCommittingThread()
        {
            std::vector<ThreadID> priority_list;
            LSQ &lsq = cpu.getLSQ();
            switch (cpu.threadPolicy)
            {
            case enums::SingleThreaded:
                return 0;
            case enums::RoundRobin:
                priority_list = cpu.roundRobinPriority(commitPriority);
                break;
            case enums::Random:
                priority_list = cpu.randomPriority();
                break;
            default:
                panic("Invalid thread policy");
            }

            for (auto tid : priority_list)
            {
                MemoryThreadInfo &mem_info = memoryInfo[tid];
                if (mem_info.inFlightInst)
                {
                    if (mem_info.inFlightInst->inLSQ)
                    {
                        if (lsq.findResponse(mem_info.inFlightInst))
                        {
                            commitPriority = tid;
                            return tid;
                        }
                    }
                    else
                    {
                        commitPriority = tid;
                        return tid;
                    }
                }
                else if (getInput(tid))
                {
                    commitPriority = tid;
                    return tid;
                }
            }

            return InvalidThreadID;
        }

        inline ThreadID
        Memory::getIssuingThread()
        {
            std::vector<ThreadID> priority_list;
            switch (cpu.threadPolicy)
            {
            case enums::SingleThreaded:
                return 0;
            case enums::RoundRobin:
                priority_list = cpu.roundRobinPriority(issuePriority);
                break;
            case enums::Random:
                priority_list = cpu.randomPriority();
                break;
            default:
                panic("Invalid thread policy");
            }

            for (auto tid : priority_list)
            {
                MemoryThreadInfo &mem_info = memoryInfo[tid];

                if (!mem_info.inFlightInst && getInput(tid))
                {
                    issuePriority = tid;
                    return tid;
                }
            }

            return InvalidThreadID;
        }

    } // namespace minor
} // namespace gem5
