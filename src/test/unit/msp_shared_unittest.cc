/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

// MSP over telemetry (telemetry/msp_shared.c) as a radio sees it: request
// chunks in, reply chunks out, in each framing version. The MSP command
// handler is a stub that records the request and answers with a reply of a
// chosen size, so the tests see only the framing.

#include <stdint.h>
#include <string.h>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "common/streambuf.h"

    #include "msp/msp.h"
    #include "msp/msp_protocol.h"

    #include "telemetry/msp_shared.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

typedef std::vector<uint8_t> bytes_t;

// What the stub saw, and what it answers with.
static int16_t stubCmd;
static bytes_t stubPayload;
static int stubReplySize;
static mspResult_e stubResult;

extern "C" {
    mspDescriptor_t mspDescriptorAlloc(void) { return 0; }

    mspResult_e mspFcProcessCommand(mspDescriptor_t srcDesc, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
    {
        UNUSED(srcDesc);
        UNUSED(mspPostProcessFn);
        stubCmd = cmd->cmd;
        stubPayload.clear();
        while (sbufBytesRemaining(&cmd->buf) > 0) {
            stubPayload.push_back(sbufReadU8(&cmd->buf));
        }
        reply->cmd = cmd->cmd;
        reply->flags = 0;
        for (int i = 0; i < stubReplySize; i++) {
            sbufWriteU8(&reply->buf, (uint8_t)(i + 1));
        }
        return stubResult;
    }
}

// The reply chunks sendMspReply() hands to the transport.
static std::vector<bytes_t> sent;

static void captureChunk(uint8_t *payload, const uint8_t payloadSize)
{
    sent.push_back(bytes_t(payload, payload + payloadSize));
}

static const uint8_t SPORT_CHUNK = 6;   // S.Port/FBUS: status + 5 bytes

static void reset(int replySize = 0)
{
    static bool initialised = false;
    if (!initialised) {
        initSharedMsp();
        initialised = true;
    }
    stubCmd = -1;
    stubPayload.clear();
    stubReplySize = replySize;
    stubResult = MSP_RESULT_ACK;
    sent.clear();
}

// Feeds one request chunk; true once a whole request was handled.
static bool feed(bytes_t chunk)
{
    return handleMspFrame(chunk.data(), chunk.size(), NULL);
}

// Collects the whole reply, one chunk per call as the telemetry driver does.
static void drainReply(uint8_t chunkSize = SPORT_CHUNK)
{
    for (int i = 0; i < 200 && sendMspReply(chunkSize, captureChunk); i++) {
    }
}

// The payload bytes of a captured reply, after a header of `headerSize`.
static bytes_t replyPayload(size_t headerSize)
{
    bytes_t out;
    for (size_t i = 0; i < sent.size(); i++) {
        const size_t from = (i == 0) ? 1 + headerSize : 1;
        out.insert(out.end(), sent[i].begin() + from, sent[i].end());
    }
    return out;
}

static uint8_t status(int version, bool start, int seq)
{
    return (version << 5) | (start ? 0x10 : 0) | (seq & 0x0F);
}

TEST(MspSharedTest, Version2RequestAndReply)
{
    reset(3);
    EXPECT_FALSE(feed({ status(2, true, 0), 0x00, 0x2C, 0x01, 0x02, 0x00 }));  // header only
    EXPECT_TRUE(feed({ status(2, false, 1), 0xAA, 0xBB }));
    EXPECT_EQ(0x012C, stubCmd);
    EXPECT_EQ(bytes_t({ 0xAA, 0xBB }), stubPayload);

    drainReply();
    ASSERT_EQ(2u, sent.size());                 // the header fills the first chunk
    EXPECT_EQ(0x50, sent[0][0] & 0x70);         // version 2, start
    EXPECT_EQ(0x00, sent[0][1]);                // flags
    EXPECT_EQ(0x2C, sent[0][2]);
    EXPECT_EQ(0x01, sent[0][3]);
    EXPECT_EQ(0x03, sent[0][4]);                // length, low byte
    EXPECT_EQ(0x00, sent[0][5]);                // length, high byte
    EXPECT_EQ(bytes_t({ 1, 2, 3 }), replyPayload(5));
}

TEST(MspSharedTest, Version3RequestCarriesPayloadInItsStartChunk)
{
    reset(0);
    EXPECT_TRUE(feed({ status(3, true, 0), 0x1E, 0x00, 0x02, 0x11, 0x22 }));
    EXPECT_EQ(0x1E, stubCmd);
    EXPECT_EQ(bytes_t({ 0x11, 0x22 }), stubPayload);
}

TEST(MspSharedTest, Version3RequestAcrossChunks)
{
    reset(0);
    EXPECT_FALSE(feed({ status(3, true, 4), 0x1E, 0x00, 0x07, 1, 2 }));
    EXPECT_TRUE(feed({ status(3, false, 5), 3, 4, 5, 6, 7 }));
    EXPECT_EQ(bytes_t({ 1, 2, 3, 4, 5, 6, 7 }), stubPayload);
}

TEST(MspSharedTest, Version3RequestWithSixteenBitLength)
{
    // 0xFF announces a 16-bit length. A client uses it from 255 bytes, but
    // the parser takes it at any length.
    reset(0);
    EXPECT_FALSE(feed({ status(3, true, 0), 0x1E, 0x00, 0xFF, 0x07, 0x00 }));
    EXPECT_FALSE(feed({ status(3, false, 1), 1, 2, 3, 4, 5 }));
    EXPECT_TRUE(feed({ status(3, false, 2), 6, 7 }));
    EXPECT_EQ(bytes_t({ 1, 2, 3, 4, 5, 6, 7 }), stubPayload);
}

TEST(MspSharedTest, Version3RequestTooBigIsAnsweredWithAnError)
{
    reset(0);
    EXPECT_TRUE(feed({ status(3, true, 0), 0x1E, 0x00, 0xFF, 0x2C, 0x01 }));  // 300 bytes
    EXPECT_EQ(-1, stubCmd);                                                    // not handled
    drainReply();
    ASSERT_GE(sent.size(), 1u);
    EXPECT_EQ(0x80, sent[0][0] & 0x80);         // error bit
    EXPECT_EQ(0x60, sent[0][0] & 0x60);         // in the request's version
    EXPECT_EQ(0x1E, sent[0][1]);                // names its command
}

TEST(MspSharedTest, Version3ShortReply)
{
    reset(11);
    feed({ status(3, true, 0), 0x1E, 0x00, 0x00 });
    drainReply();
    ASSERT_EQ(3u, sent.size());                 // 2 + 5 + 4 payload bytes
    EXPECT_EQ(0x70, sent[0][0] & 0x70);
    EXPECT_EQ(0x1E, sent[0][1]);
    EXPECT_EQ(0x00, sent[0][2]);
    EXPECT_EQ(11, sent[0][3]);
    bytes_t payload = replyPayload(3);
    ASSERT_EQ(11u, payload.size());
    EXPECT_EQ(1, payload[0]);
    EXPECT_EQ(11, payload[10]);
    for (size_t i = 1; i < sent.size(); i++) {
        EXPECT_EQ(0, sent[i][0] & 0x10);        // no start bit after the first
        EXPECT_EQ(((sent[0][0] & 0x0F) + i) & 0x0F, sent[i][0] & 0x0Fu);
    }
}

TEST(MspSharedTest, Version3ReplyEndingOnAChunkBoundary)
{
    // 2 + 5 bytes fill two chunks exactly; a status-only chunk follows.
    reset(7);
    feed({ status(3, true, 0), 0x1E, 0x00, 0x00 });
    drainReply();
    ASSERT_EQ(3u, sent.size());
    EXPECT_EQ(1u, sent[2].size());
    EXPECT_EQ(7u, replyPayload(3).size());
}

TEST(MspSharedTest, Version3LongReplyUsesSixteenBitLength)
{
    reset(300);
    feed({ status(3, true, 0), 0x1E, 0x00, 0x00 });
    drainReply();
    ASSERT_GE(sent.size(), 2u);
    EXPECT_EQ(0x1E, sent[0][1]);
    EXPECT_EQ(0xFF, sent[0][3]);
    EXPECT_EQ(0x2C, sent[0][4]);
    EXPECT_EQ(0x01, sent[0][5]);
    bytes_t payload = replyPayload(5);
    ASSERT_GE(payload.size(), 300u);
    EXPECT_EQ(1, payload[0]);
    EXPECT_EQ(300 & 0xFF, payload[299]);
}

TEST(MspSharedTest, Version1StillWorks)
{
    reset(2);
    EXPECT_TRUE(feed({ status(1, true, 0), 0x01, 0x20, 0x55, 0x74 }));  // length 1, command 32, payload, CRC
    EXPECT_EQ(32, stubCmd);
    EXPECT_EQ(bytes_t({ 0x55 }), stubPayload);
    drainReply();
    ASSERT_GE(sent.size(), 1u);
    EXPECT_EQ(0x30, sent[0][0] & 0x70);
    EXPECT_EQ(2, sent[0][1]);                   // length
    EXPECT_EQ(32, sent[0][2]);                  // command
}

TEST(MspSharedTest, ReplacementRequestStartsWithItsOwnStartChunk)
{
    // A request that arrives while a long reply is going out replaces it,
    // and the new reply begins with a START chunk.
    reset(40);
    feed({ status(3, true, 0), 0x1E, 0x00, 0x00 });
    sendMspReply(SPORT_CHUNK, captureChunk);
    sendMspReply(SPORT_CHUNK, captureChunk);
    sent.clear();
    stubReplySize = 3;
    feed({ status(3, true, 1), 0x1F, 0x00, 0x00 });
    drainReply();
    ASSERT_EQ(2u, sent.size());                 // 2 + 1 payload bytes
    EXPECT_EQ(0x10, sent[0][0] & 0x10);
    EXPECT_EQ(0x1F, sent[0][1]);
    EXPECT_EQ(3, sent[0][3]);
    EXPECT_EQ(bytes_t({ 1, 2, 3 }), replyPayload(3));
}
