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

#include "cpu/minor/decode.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/minor/pipeline.hh"
#include "debug/Decode.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Minor, minor);
namespace minor
{

Decode::Decode(const std::string &name,
    MinorCPU &cpu_,
    const BaseMinorCPUParams &params,
    Latch<ForwardInstData>::Output inp_,
    Latch<ForwardInstData>::Input out_,
    std::vector<InputBuffer<ForwardInstData>> &next_stage_input_buffer) :
    Named(name),
    cpu(cpu_),
    inp(inp_),
    out(out_),
    nextStageReserve(next_stage_input_buffer),
    outputWidth(params.executeInputWidth),
    processMoreThanOneInput(params.decodeCycleInput),
    decodeInfo(params.numThreads),
    threadPriority(0)
{
    if (outputWidth < 1)
        fatal("%s: executeInputWidth must be >= 1 (%d)\n", name, outputWidth);

    if (params.decodeInputBufferSize < 1) {
        fatal("%s: decodeInputBufferSize must be >= 1 (%d)\n", name,
        params.decodeInputBufferSize);
    }

    /* Per-thread input buffers */
    for (ThreadID tid = 0; tid < params.numThreads; tid++) {
        inputBuffer.push_back(
            InputBuffer<ForwardInstData>(
                name + ".inputBuffer" + std::to_string(tid), "insts",
                params.decodeInputBufferSize));
    }
}

const ForwardInstData *
Decode::getInput(ThreadID tid)
{
    /* Get insts from the inputBuffer to work with */
    if (!inputBuffer[tid].empty()) {
        const ForwardInstData &head = inputBuffer[tid].front();

        return (head.isBubble() ? NULL : &(inputBuffer[tid].front()));
    } else {
        return NULL;
    }
}

void
Decode::popInput(ThreadID tid)
{
    if (!inputBuffer[tid].empty())
        inputBuffer[tid].pop();

    decodeInfo[tid].inputIndex = 0;
    decodeInfo[tid].inMacroop = false;
}

#if TRACING_ON
/** Add the tracing data to an instruction.  This originates in
 *  decode because this is the first place that execSeqNums are known
 *  (these are used as the 'FetchSeq' in tracing data) */
static void
dynInstAddTracing(MinorDynInstPtr inst, StaticInstPtr static_inst,
    MinorCPU &cpu)
{
    inst->traceData = cpu.getTracer()->getInstRecord(curTick(),
        cpu.getContext(inst->id.threadId),
        inst->staticInst, *inst->pc, static_inst);

    /* Use the execSeqNum as the fetch sequence number as this most closely
     *  matches the other processor models' idea of fetch sequence */
    if (inst->traceData)
        inst->traceData->setFetchSeq(inst->id.execSeqNum);
}
#endif

void
Decode::pushIntoInpBuffer()
{
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].setTail(*inp.outputWire);
}

void
Decode::updateAllThreadsStatus()
{
    for (ThreadID tid = 0; tid < cpu.numThreads; tid++)
        decodeInfo[tid].blocked = !nextStageReserve[tid].canReserve();
}

void
Decode::advanceInput(ThreadID tid)
{
    decodeInfo[tid].inputIndex++;
    decodeInfo[tid].inMacroop = false;
}

void
Decode::setUpPcForMicroop(ThreadID tid, MinorDynInstPtr inst)
{
    if (!decodeInfo[tid].inMacroop) {
        set(decodeInfo[tid].microopPC, *inst->pc);
        decodeInfo[tid].inMacroop = true;
    }
}

StaticInstPtr
Decode::extractMicroInst(ThreadID tid, StaticInstPtr static_inst)
{
    return static_inst->fetchMicroop(
                                decodeInfo[tid].microopPC->microPC());
}

MinorDynInstPtr
Decode::packInst(StaticInstPtr static_micro_inst, InstId id, ThreadID tid)
{
    MinorDynInstPtr output_inst = NULL;

    output_inst = new MinorDynInst(static_micro_inst, id);
    set(output_inst->pc, decodeInfo[tid].microopPC);
    output_inst->fault = NoFault;

    return output_inst;
}

void
Decode::allowPredictionOnLastMicroop(
    StaticInstPtr static_micro_inst,
    MinorDynInstPtr output_inst,
    MinorDynInstPtr macro_inst)
{
    if (!static_micro_inst->isLastMicroop()) return;

    output_inst->predictedTaken = macro_inst->predictedTaken;
    set(output_inst->predictedTarget,
            macro_inst->predictedTarget);
}

MinorDynInstPtr
Decode::decomposition(
    ThreadID tid,
    MinorDynInstPtr inst,
    unsigned int output_index)
{
    /* Generate a new micro-op */
    StaticInstPtr static_micro_inst;
    StaticInstPtr static_inst = inst->staticInst;

    /* Set up PC for the next micro-op emitted */
    setUpPcForMicroop(tid, inst);

    /* Get the micro-op static instruction from the
        * static_inst. */
    static_micro_inst = extractMicroInst(tid, static_inst);

    MinorDynInstPtr output_inst = packInst(static_micro_inst, inst->id, tid);

    /* Allow a predicted next address only on the last
        *  microop */
    allowPredictionOnLastMicroop(static_micro_inst, output_inst, inst);

    DPRINTF(Decode, "Microop decomposition inputIndex:"
        " %d output_index: %d lastMicroop: %s microopPC:"
        " %s inst: %d\n",
        decodeInfo[tid].inputIndex, output_index,
        (static_micro_inst->isLastMicroop() ?
            "true" : "false"),
        *decodeInfo[tid].microopPC,
        *output_inst);

    static_micro_inst->advancePC(*decodeInfo[tid].microopPC);

    /* Step input if this is the last micro-op */
    if (static_micro_inst->isLastMicroop()) {
        advanceInput(tid);
    }
    return output_inst;
}

void
Decode::assignExecSeqNum(ThreadID tid, MinorDynInstPtr output_inst)
{
    output_inst->id.execSeqNum = decodeInfo[tid].execSeqNum;
    /* Step to next sequence number */
    decodeInfo[tid].execSeqNum++;
}

void
Decode::packIntoOutput(
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

const ForwardInstData *
Decode::maybeMoreInput(ThreadID tid, const ForwardInstData *insts_in)
{
    if (decodeInfo[tid].inputIndex == insts_in->width()) {
        /* If we have just been producing micro-ops, we *must* have
            * got to the end of that for inputIndex to be pushed past
            * insts_in->width() */
        assert(!decodeInfo[tid].inMacroop);
        popInput(tid);
        insts_in = NULL;

        if (processMoreThanOneInput) {
            DPRINTF(Decode, "Wrapping\n");
            insts_in = getInput(tid);
        }
    }

    return insts_in;
}

void
Decode::reserveSpaceInNextStage(ForwardInstData &insts_out, ThreadID tid)
{
    if (!insts_out.isBubble()) {
        /* Note activity of following buffer */
        cpu.activityRecorder->activity();
        insts_out.threadId = tid;
        nextStageReserve[tid].reserve();
    }

}

void
Decode::markStageActivity()
{
    for (ThreadID i = 0; i < cpu.numThreads; i++)
    {
        if (getInput(i) && nextStageReserve[i].canReserve()) {
            cpu.activityRecorder->activateStage(Pipeline::DecodeStageId);
            break;
        }
    }
}

void
Decode::pushTailInpBuffer()
{
    if (!inp.outputWire->isBubble())
        inputBuffer[inp.outputWire->threadId].pushTail();

}

void
Decode::evaluate()
{
    /* Push input onto appropriate input buffer */
    pushIntoInpBuffer();

    ForwardInstData &insts_out = *out.inputWire;

    assert(insts_out.isBubble());

    /* Check if each thread can reserve in next stage */
    updateAllThreadsStatus();

    ThreadID tid = getScheduledThread();

    if (tid != InvalidThreadID)
    {
        DecodeThreadInfo &decode_info = decodeInfo[tid];
        const ForwardInstData *insts_in = getInput(tid);

        unsigned int output_index = 0;

        /* Pack instructions into the output while we can.  This may involve
         * using more than one input line */
        while (insts_in &&
           decode_info.inputIndex < insts_in->width() && /* Still more input */
           output_index < outputWidth /* Still more output to fill */)
        {
            MinorDynInstPtr inst = insts_in->insts[decode_info.inputIndex];

            if (inst->isBubble()) {
                /* Skip */
                advanceInput(tid);
            } else {
                StaticInstPtr static_inst = inst->staticInst;
                /* Static inst of a macro-op above the output_inst */
                StaticInstPtr parent_static_inst = NULL;
                MinorDynInstPtr output_inst = inst;

                if (inst->isFault()) {
                    DPRINTF(Decode, "Fault being passed: %d\n",
                            inst->fault->name());
                    advanceInput(tid);

                } else if (!static_inst->isMacroop()) {
                    /* Doesn't need decomposing, pass on instruction */
                    DPRINTF(Decode, "Passing on inst: %s inputIndex:"
                        " %d output_index: %d\n",
                        *output_inst, decode_info.inputIndex, output_index);
                    parent_static_inst = static_inst;
                    /* Step input */
                    advanceInput(tid);

                } else {
                    /* It is a macroop to decompose */
                    output_inst = decomposition(tid, inst, output_index);
                    /* Acknowledge that the static_inst isn't mine, it's my
                     * parent macro-op's */
                    parent_static_inst = static_inst;
                }

                /* Add tracing */
#if TRACING_ON
                dynInstAddTracing(output_inst, parent_static_inst, cpu);
#endif

                /* Set execSeqNum of output_inst and step to next
                *  sequence number */
                assignExecSeqNum(tid, output_inst);

                packIntoOutput(output_inst, insts_out, &output_index);

            }

            /* Have we finished with the input? */
            insts_in = maybeMoreInput(tid, insts_in);
        }

        /* The rest of the output (if any) should already have been packed
         *  with bubble instructions by insts_out's initialisation
         *
         *  for (; output_index < outputWidth; output_index++)
         *      assert(insts_out.insts[output_index]->isBubble());
         */
    }

    /* If we generated output, reserve space for the result in the next stage
     *  and mark the stage as being active this cycle */
    reserveSpaceInNextStage(insts_out, tid);

    /* If we still have input to process and somewhere to put it,
     *  mark stage as active */
    markStageActivity();

    /* Make sure the input (if any left) is pushed */
    pushTailInpBuffer();

}

inline ThreadID
Decode::getScheduledThread()
{
    /* Select thread via policy. */
    std::vector<ThreadID> priority_list;

    switch (cpu.threadPolicy) {
      case enums::SingleThreaded:
        priority_list.push_back(0);
        break;
      case enums::RoundRobin:
        priority_list = cpu.roundRobinPriority(threadPriority);
        break;
      case enums::Random:
        priority_list = cpu.randomPriority();
        break;
      default:
        panic("Unknown fetch policy");
    }

    for (auto tid : priority_list) {
        if (getInput(tid) && !decodeInfo[tid].blocked) {
            threadPriority = tid;
            return tid;
        }
    }

   return InvalidThreadID;
}

bool
Decode::isDrained()
{
    for (const auto &buffer : inputBuffer) {
        if (!buffer.empty())
            return false;
    }

    return (*inp.outputWire).isBubble();
}

void
Decode::minorTrace() const
{
    std::ostringstream data;

    if (decodeInfo[0].blocked)
        data << 'B';
    else
        (*out.inputWire).reportData(data);

    minor::minorTrace("insts=%s\n", data.str());
    inputBuffer[0].minorTrace();
}

} // namespace minor
} // namespace gem5
