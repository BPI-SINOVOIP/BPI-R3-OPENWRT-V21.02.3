package wire

import (
	"bytes"
	"time"

	"github.com/lucas-clemente/quic-go/internal/protocol"
	"github.com/lucas-clemente/quic-go/internal/qerr"
	. "github.com/onsi/ginkgo"
	. "github.com/onsi/gomega"
)

var _ = Describe("Frame parsing", func() {
	var (
		buf    *bytes.Buffer
		parser FrameParser
	)

	BeforeEach(func() {
		buf = &bytes.Buffer{}
		parser = NewFrameParser(true, versionIETFFrames)
	})

	It("returns nil if there's nothing more to read", func() {
		f, err := parser.ParseNext(bytes.NewReader(nil), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(f).To(BeNil())
	})

	It("skips PADDING frames", func() {
		buf.Write([]byte{0}) // PADDING frame
		(&PingFrame{}).Write(buf, versionIETFFrames)
		f, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(f).To(Equal(&PingFrame{}))
	})

	It("handles PADDING at the end", func() {
		r := bytes.NewReader([]byte{0, 0, 0})
		f, err := parser.ParseNext(r, protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(f).To(BeNil())
		Expect(r.Len()).To(BeZero())
	})

	It("unpacks ACK frames", func() {
		f := &AckFrame{AckRanges: []AckRange{{Smallest: 1, Largest: 0x13}}}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).ToNot(BeNil())
		Expect(frame).To(BeAssignableToTypeOf(f))
		Expect(frame.(*AckFrame).LargestAcked()).To(Equal(protocol.PacketNumber(0x13)))
	})

	It("uses the custom ack delay exponent for 1RTT packets", func() {
		parser.SetAckDelayExponent(protocol.AckDelayExponent + 2)
		f := &AckFrame{
			AckRanges: []AckRange{{Smallest: 1, Largest: 1}},
			DelayTime: time.Second,
		}
		Expect(f.Write(buf, versionIETFFrames)).To(Succeed())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		// The ACK frame is always written using the protocol.AckDelayExponent.
		// That's why we expect a different value when parsing.
		Expect(frame.(*AckFrame).DelayTime).To(Equal(4 * time.Second))
	})

	It("uses the default ack delay exponent for non-1RTT packets", func() {
		parser.SetAckDelayExponent(protocol.AckDelayExponent + 2)
		f := &AckFrame{
			AckRanges: []AckRange{{Smallest: 1, Largest: 1}},
			DelayTime: time.Second,
		}
		Expect(f.Write(buf, versionIETFFrames)).To(Succeed())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.EncryptionHandshake)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame.(*AckFrame).DelayTime).To(Equal(time.Second))
	})

	It("unpacks RESET_STREAM frames", func() {
		f := &ResetStreamFrame{
			StreamID:  0xdeadbeef,
			FinalSize: 0xdecafbad1234,
			ErrorCode: 0x1337,
		}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks STOP_SENDING frames", func() {
		f := &StopSendingFrame{StreamID: 0x42}
		buf := &bytes.Buffer{}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks CRYPTO frames", func() {
		f := &CryptoFrame{
			Offset: 0x1337,
			Data:   []byte("lorem ipsum"),
		}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).ToNot(BeNil())
		Expect(frame).To(Equal(f))
	})

	It("unpacks NEW_TOKEN frames", func() {
		f := &NewTokenFrame{Token: []byte("foobar")}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).ToNot(BeNil())
		Expect(frame).To(Equal(f))
	})

	It("unpacks STREAM frames", func() {
		f := &StreamFrame{
			StreamID: 0x42,
			Offset:   0x1337,
			Fin:      true,
			Data:     []byte("foobar"),
		}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).ToNot(BeNil())
		Expect(frame).To(Equal(f))
	})

	It("unpacks MAX_DATA frames", func() {
		f := &MaxDataFrame{
			MaximumData: 0xcafe,
		}
		buf := &bytes.Buffer{}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks MAX_STREAM_DATA frames", func() {
		f := &MaxStreamDataFrame{
			StreamID:          0xdeadbeef,
			MaximumStreamData: 0xdecafbad,
		}
		buf := &bytes.Buffer{}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks MAX_STREAMS frames", func() {
		f := &MaxStreamsFrame{
			Type:         protocol.StreamTypeBidi,
			MaxStreamNum: 0x1337,
		}
		buf := &bytes.Buffer{}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks DATA_BLOCKED frames", func() {
		f := &DataBlockedFrame{MaximumData: 0x1234}
		buf := &bytes.Buffer{}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks STREAM_DATA_BLOCKED frames", func() {
		f := &StreamDataBlockedFrame{
			StreamID:          0xdeadbeef,
			MaximumStreamData: 0xdead,
		}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks STREAMS_BLOCKED frames", func() {
		f := &StreamsBlockedFrame{
			Type:        protocol.StreamTypeBidi,
			StreamLimit: 0x1234567,
		}
		buf := &bytes.Buffer{}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks NEW_CONNECTION_ID frames", func() {
		f := &NewConnectionIDFrame{
			SequenceNumber:      0x1337,
			ConnectionID:        protocol.ConnectionID{0xde, 0xad, 0xbe, 0xef},
			StatelessResetToken: protocol.StatelessResetToken{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
		}
		buf := &bytes.Buffer{}
		Expect(f.Write(buf, versionIETFFrames)).To(Succeed())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks RETIRE_CONNECTION_ID frames", func() {
		f := &RetireConnectionIDFrame{SequenceNumber: 0x1337}
		buf := &bytes.Buffer{}
		Expect(f.Write(buf, versionIETFFrames)).To(Succeed())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks PATH_CHALLENGE frames", func() {
		f := &PathChallengeFrame{Data: [8]byte{1, 2, 3, 4, 5, 6, 7, 8}}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).ToNot(BeNil())
		Expect(frame).To(BeAssignableToTypeOf(f))
		Expect(frame.(*PathChallengeFrame).Data).To(Equal([8]byte{1, 2, 3, 4, 5, 6, 7, 8}))
	})

	It("unpacks PATH_RESPONSE frames", func() {
		f := &PathResponseFrame{Data: [8]byte{1, 2, 3, 4, 5, 6, 7, 8}}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).ToNot(BeNil())
		Expect(frame).To(BeAssignableToTypeOf(f))
		Expect(frame.(*PathResponseFrame).Data).To(Equal([8]byte{1, 2, 3, 4, 5, 6, 7, 8}))
	})

	It("unpacks CONNECTION_CLOSE frames", func() {
		f := &ConnectionCloseFrame{
			IsApplicationError: true,
			ReasonPhrase:       "foobar",
		}
		buf := &bytes.Buffer{}
		err := f.Write(buf, versionIETFFrames)
		Expect(err).ToNot(HaveOccurred())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks HANDSHAKE_DONE frames", func() {
		f := &HandshakeDoneFrame{}
		buf := &bytes.Buffer{}
		Expect(f.Write(buf, versionIETFFrames)).To(Succeed())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("unpacks DATAGRAM frames", func() {
		f := &DatagramFrame{Data: []byte("foobar")}
		buf := &bytes.Buffer{}
		Expect(f.Write(buf, versionIETFFrames)).To(Succeed())
		frame, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).ToNot(HaveOccurred())
		Expect(frame).To(Equal(f))
	})

	It("errors when DATAGRAM frames are not supported", func() {
		parser = NewFrameParser(false, versionIETFFrames)
		f := &DatagramFrame{Data: []byte("foobar")}
		buf := &bytes.Buffer{}
		Expect(f.Write(buf, versionIETFFrames)).To(Succeed())
		_, err := parser.ParseNext(bytes.NewReader(buf.Bytes()), protocol.Encryption1RTT)
		Expect(err).To(MatchError(&qerr.TransportError{
			ErrorCode:    qerr.FrameEncodingError,
			FrameType:    0x30,
			ErrorMessage: "unknown frame type",
		}))
	})

	It("errors on invalid type", func() {
		_, err := parser.ParseNext(bytes.NewReader([]byte{0x42}), protocol.Encryption1RTT)
		Expect(err).To(MatchError(&qerr.TransportError{
			ErrorCode:    qerr.FrameEncodingError,
			FrameType:    0x42,
			ErrorMessage: "unknown frame type",
		}))
	})

	It("errors on invalid frames", func() {
		f := &MaxStreamDataFrame{
			StreamID:          0x1337,
			MaximumStreamData: 0xdeadbeef,
		}
		b := &bytes.Buffer{}
		f.Write(b, versionIETFFrames)
		_, err := parser.ParseNext(bytes.NewReader(b.Bytes()[:b.Len()-2]), protocol.Encryption1RTT)
		Expect(err).To(HaveOccurred())
		Expect(err.(*qerr.TransportError).ErrorCode).To(Equal(qerr.FrameEncodingError))
	})

	Context("encryption level check", func() {
		frames := []Frame{
			&PingFrame{},
			&AckFrame{AckRanges: []AckRange{{Smallest: 1, Largest: 42}}},
			&ResetStreamFrame{},
			&StopSendingFrame{},
			&CryptoFrame{},
			&NewTokenFrame{Token: []byte("lorem ipsum")},
			&StreamFrame{Data: []byte("foobar")},
			&MaxDataFrame{},
			&MaxStreamDataFrame{},
			&MaxStreamsFrame{},
			&DataBlockedFrame{},
			&StreamDataBlockedFrame{},
			&StreamsBlockedFrame{},
			&NewConnectionIDFrame{ConnectionID: protocol.ConnectionID{0xde, 0xad, 0xbe, 0xef}},
			&RetireConnectionIDFrame{},
			&PathChallengeFrame{},
			&PathResponseFrame{},
			&ConnectionCloseFrame{},
			&HandshakeDoneFrame{},
			&DatagramFrame{},
		}

		var framesSerialized [][]byte

		BeforeEach(func() {
			framesSerialized = nil
			for _, frame := range frames {
				buf := &bytes.Buffer{}
				Expect(frame.Write(buf, versionIETFFrames)).To(Succeed())
				framesSerialized = append(framesSerialized, buf.Bytes())
			}
		})

		It("rejects all frames but ACK, CRYPTO, PING and CONNECTION_CLOSE in Initial packets", func() {
			for i, b := range framesSerialized {
				_, err := parser.ParseNext(bytes.NewReader(b), protocol.EncryptionInitial)
				switch frames[i].(type) {
				case *AckFrame, *ConnectionCloseFrame, *CryptoFrame, *PingFrame:
					Expect(err).ToNot(HaveOccurred())
				default:
					Expect(err).To(BeAssignableToTypeOf(&qerr.TransportError{}))
					Expect(err.(*qerr.TransportError).ErrorCode).To(Equal(qerr.FrameEncodingError))
					Expect(err.(*qerr.TransportError).ErrorMessage).To(ContainSubstring("not allowed at encryption level Initial"))
				}
			}
		})

		It("rejects all frames but ACK, CRYPTO, PING and CONNECTION_CLOSE in Handshake packets", func() {
			for i, b := range framesSerialized {
				_, err := parser.ParseNext(bytes.NewReader(b), protocol.EncryptionHandshake)
				switch frames[i].(type) {
				case *AckFrame, *ConnectionCloseFrame, *CryptoFrame, *PingFrame:
					Expect(err).ToNot(HaveOccurred())
				default:
					Expect(err).To(BeAssignableToTypeOf(&qerr.TransportError{}))
					Expect(err.(*qerr.TransportError).ErrorCode).To(Equal(qerr.FrameEncodingError))
					Expect(err.(*qerr.TransportError).ErrorMessage).To(ContainSubstring("not allowed at encryption level Handshake"))
				}
			}
		})

		It("rejects all frames but ACK, CRYPTO, CONNECTION_CLOSE, NEW_TOKEN, PATH_RESPONSE and RETIRE_CONNECTION_ID in 0-RTT packets", func() {
			for i, b := range framesSerialized {
				_, err := parser.ParseNext(bytes.NewReader(b), protocol.Encryption0RTT)
				switch frames[i].(type) {
				case *AckFrame, *ConnectionCloseFrame, *CryptoFrame, *NewTokenFrame, *PathResponseFrame, *RetireConnectionIDFrame:
					Expect(err).To(BeAssignableToTypeOf(&qerr.TransportError{}))
					Expect(err.(*qerr.TransportError).ErrorCode).To(Equal(qerr.FrameEncodingError))
					Expect(err.(*qerr.TransportError).ErrorMessage).To(ContainSubstring("not allowed at encryption level 0-RTT"))
				default:
					Expect(err).ToNot(HaveOccurred())
				}
			}
		})

		It("accepts all frame types in 1-RTT packets", func() {
			for _, b := range framesSerialized {
				_, err := parser.ParseNext(bytes.NewReader(b), protocol.Encryption1RTT)
				Expect(err).ToNot(HaveOccurred())
			}
		})
	})
})
