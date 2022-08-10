package qtls

import (
	"bytes"
	"fmt"
	"net"
	"testing"
	"time"
)

type recordLayer struct {
	in  <-chan []byte
	out chan<- []byte

	alertSent alert
}

func (r *recordLayer) SetReadKey(encLevel EncryptionLevel, suite *CipherSuiteTLS13, trafficSecret []byte) {
}
func (r *recordLayer) SetWriteKey(encLevel EncryptionLevel, suite *CipherSuiteTLS13, trafficSecret []byte) {
}
func (r *recordLayer) ReadHandshakeMessage() ([]byte, error) { return <-r.in, nil }
func (r *recordLayer) WriteRecord(b []byte) (int, error)     { r.out <- b; return len(b), nil }
func (r *recordLayer) SendAlert(a uint8)                     { r.alertSent = alert(a) }

type exportedKey struct {
	typ           string // "read" or "write"
	encLevel      EncryptionLevel
	suite         *CipherSuiteTLS13
	trafficSecret []byte
}

func compareExportedKeys(t *testing.T, k1, k2 *exportedKey) {
	if k1.encLevel != k2.encLevel || k1.suite.ID != k2.suite.ID || !bytes.Equal(k1.trafficSecret, k2.trafficSecret) {
		t.Fatal("mismatching keys")
	}
}

type recordLayerWithKeys struct {
	in  <-chan []byte
	out chan<- interface{}
}

func (r *recordLayerWithKeys) SetReadKey(encLevel EncryptionLevel, suite *CipherSuiteTLS13, trafficSecret []byte) {
	r.out <- &exportedKey{typ: "read", encLevel: encLevel, suite: suite, trafficSecret: trafficSecret}
}
func (r *recordLayerWithKeys) SetWriteKey(encLevel EncryptionLevel, suite *CipherSuiteTLS13, trafficSecret []byte) {
	r.out <- &exportedKey{typ: "write", encLevel: encLevel, suite: suite, trafficSecret: trafficSecret}
}
func (r *recordLayerWithKeys) ReadHandshakeMessage() ([]byte, error) { return <-r.in, nil }
func (r *recordLayerWithKeys) WriteRecord(b []byte) (int, error)     { r.out <- b; return len(b), nil }
func (r *recordLayerWithKeys) SendAlert(uint8)                       {}

type unusedConn struct {
	remoteAddr net.Addr
}

var _ net.Conn = &unusedConn{}

func (unusedConn) Read([]byte) (int, error)         { panic("unexpected call to Read()") }
func (unusedConn) Write([]byte) (int, error)        { panic("unexpected call to Write()") }
func (unusedConn) Close() error                     { return nil }
func (unusedConn) LocalAddr() net.Addr              { return &net.TCPAddr{} }
func (c *unusedConn) RemoteAddr() net.Addr          { return c.remoteAddr }
func (unusedConn) SetDeadline(time.Time) error      { return nil }
func (unusedConn) SetReadDeadline(time.Time) error  { return nil }
func (unusedConn) SetWriteDeadline(time.Time) error { return nil }

func TestAlternativeRecordLayer(t *testing.T) {
	sIn := make(chan []byte, 10)
	sOut := make(chan interface{}, 10)
	defer close(sOut)
	cIn := make(chan []byte, 10)
	cOut := make(chan interface{}, 10)
	defer close(cOut)

	serverEvents := make(chan interface{}, 100)
	go func() {
		for {
			c, ok := <-sOut
			if !ok {
				return
			}
			serverEvents <- c
			if b, ok := c.([]byte); ok {
				cIn <- b
			}
		}
	}()

	clientEvents := make(chan interface{}, 100)
	go func() {
		for {
			c, ok := <-cOut
			if !ok {
				return
			}
			clientEvents <- c
			if b, ok := c.([]byte); ok {
				sIn <- b
			}
		}
	}()

	errChan := make(chan error)
	go func() {
		extraConf := &ExtraConfig{
			AlternativeRecordLayer: &recordLayerWithKeys{in: sIn, out: sOut},
		}
		tlsConn := Server(&unusedConn{}, testConfig, extraConf)
		defer tlsConn.Close()
		errChan <- tlsConn.Handshake()
	}()

	extraConf := &ExtraConfig{
		AlternativeRecordLayer: &recordLayerWithKeys{in: cIn, out: cOut},
	}
	tlsConn := Client(&unusedConn{}, testConfig, extraConf)
	defer tlsConn.Close()
	if err := tlsConn.Handshake(); err != nil {
		t.Fatalf("Handshake failed: %s", err)
	}

	// Handshakes completed. Now check that events were received in the correct order.
	var clientHandshakeReadKey, clientHandshakeWriteKey *exportedKey
	var clientApplicationReadKey, clientApplicationWriteKey *exportedKey
	for i := 0; i <= 5; i++ {
		ev := <-clientEvents
		switch i {
		case 0:
			if ev.([]byte)[0] != typeClientHello {
				t.Fatalf("expected ClientHello")
			}
		case 1:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "write" || keyEv.encLevel != EncryptionHandshake {
				t.Fatalf("expected the handshake write key")
			}
			clientHandshakeWriteKey = keyEv
		case 2:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "read" || keyEv.encLevel != EncryptionHandshake {
				t.Fatalf("expected the handshake read key")
			}
			clientHandshakeReadKey = keyEv
		case 3:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "read" || keyEv.encLevel != EncryptionApplication {
				t.Fatalf("expected the application read key")
			}
			clientApplicationReadKey = keyEv
		case 4:
			if ev.([]byte)[0] != typeFinished {
				t.Fatalf("expected Finished")
			}
		case 5:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "write" || keyEv.encLevel != EncryptionApplication {
				t.Fatalf("expected the application write key")
			}
			clientApplicationWriteKey = keyEv
		}
	}
	if len(clientEvents) > 0 {
		t.Fatal("didn't expect any more client events")
	}

	for i := 0; i <= 8; i++ {
		ev := <-serverEvents
		switch i {
		case 0:
			if ev.([]byte)[0] != typeServerHello {
				t.Fatalf("expected ServerHello")
			}
		case 1:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "read" || keyEv.encLevel != EncryptionHandshake {
				t.Fatalf("expected the handshake read key")
			}
			compareExportedKeys(t, clientHandshakeWriteKey, keyEv)
		case 2:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "write" || keyEv.encLevel != EncryptionHandshake {
				t.Fatalf("expected the handshake write key")
			}
			compareExportedKeys(t, clientHandshakeReadKey, keyEv)
		case 3:
			if ev.([]byte)[0] != typeEncryptedExtensions {
				t.Fatalf("expected EncryptedExtensions")
			}
		case 4:
			if ev.([]byte)[0] != typeCertificate {
				t.Fatalf("expected Certificate")
			}
		case 5:
			if ev.([]byte)[0] != typeCertificateVerify {
				t.Fatalf("expected CertificateVerify")
			}
		case 6:
			if ev.([]byte)[0] != typeFinished {
				t.Fatalf("expected Finished")
			}
		case 7:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "write" || keyEv.encLevel != EncryptionApplication {
				t.Fatalf("expected the application write key")
			}
			compareExportedKeys(t, clientApplicationReadKey, keyEv)
		case 8:
			keyEv := ev.(*exportedKey)
			if keyEv.typ != "read" || keyEv.encLevel != EncryptionApplication {
				t.Fatalf("expected the application read key")
			}
			compareExportedKeys(t, clientApplicationWriteKey, keyEv)
		}
	}
	if len(serverEvents) > 0 {
		t.Fatal("didn't expect any more server events")
	}
}

func TestErrorOnOldTLSVersions(t *testing.T) {
	sIn := make(chan []byte, 10)
	cIn := make(chan []byte, 10)
	cOut := make(chan []byte, 10)

	go func() {
		for {
			b, ok := <-cOut
			if !ok {
				return
			}
			if b[0] == typeClientHello {
				m := new(clientHelloMsg)
				if !m.unmarshal(b) {
					panic("unmarshal failed")
				}
				m.raw = nil // need to reset, so marshal() actually marshals the changes
				m.supportedVersions = []uint16{VersionTLS11, VersionTLS13}
				b = m.marshal()
			}
			sIn <- b
		}
	}()

	done := make(chan struct{})
	go func() {
		defer close(done)
		extraConf := &ExtraConfig{AlternativeRecordLayer: &recordLayer{in: cIn, out: cOut}}
		Client(&unusedConn{}, testConfig, extraConf).Handshake()
	}()

	serverRecordLayer := &recordLayer{in: sIn, out: cIn}
	extraConf := &ExtraConfig{AlternativeRecordLayer: serverRecordLayer}
	tlsConn := Server(&unusedConn{}, testConfig, extraConf)
	defer tlsConn.Close()
	err := tlsConn.Handshake()
	if err == nil || err.Error() != "tls: client offered old TLS version 0x302" {
		t.Fatal("expected the server to error when the client offers old versions")
	}
	if serverRecordLayer.alertSent != alertProtocolVersion {
		t.Fatal("expected a protocol version alert to be sent")
	}

	cIn <- []byte{'f'}
	<-done
}

func TestRejectConfigWithOldMaxVersion(t *testing.T) {
	t.Run("for the client", func(t *testing.T) {
		config := testConfig.Clone()
		config.MaxVersion = VersionTLS12
		tlsConn := Client(&unusedConn{}, config, &ExtraConfig{AlternativeRecordLayer: &recordLayer{}})
		err := tlsConn.Handshake()
		if err == nil || err.Error() != "tls: MaxVersion prevents QUIC from using TLS 1.3" {
			t.Errorf("expected the handshake to fail")
		}
	})

	t.Run("for the server", func(t *testing.T) {
		in := make(chan []byte, 10)
		out := make(chan []byte, 10)

		done := make(chan struct{})
		go func() {
			defer close(done)
			Client(
				&unusedConn{},
				testConfig,
				&ExtraConfig{AlternativeRecordLayer: &recordLayer{in: in, out: out}},
			).Handshake()
		}()

		config := testConfig.Clone()
		config.MaxVersion = VersionTLS12
		serverRecordLayer := &recordLayer{in: out, out: in}
		err := Server(
			&unusedConn{},
			config,
			&ExtraConfig{AlternativeRecordLayer: serverRecordLayer},
		).Handshake()
		if err == nil || err.Error() != "tls: MaxVersion prevents QUIC from using TLS 1.3" {
			t.Errorf("expected the handshake to fail")
		}
		if serverRecordLayer.alertSent != alertInternalError {
			t.Fatal("expected an internal error alert to be sent")
		}
	})

	t.Run("for the server (using GetConfigForClient)", func(t *testing.T) {
		in := make(chan []byte, 10)
		out := make(chan []byte, 10)

		done := make(chan struct{})
		go func() {
			defer close(done)
			Client(
				&unusedConn{},
				testConfig,
				&ExtraConfig{AlternativeRecordLayer: &recordLayer{in: in, out: out}},
			).Handshake()
		}()

		config := testConfig.Clone()
		config.GetConfigForClient = func(*ClientHelloInfo) (*Config, error) {
			conf := testConfig.Clone()
			conf.MaxVersion = VersionTLS12
			return conf, nil
		}
		serverRecordLayer := &recordLayer{in: out, out: in}
		err := Server(
			&unusedConn{},
			config,
			&ExtraConfig{AlternativeRecordLayer: serverRecordLayer},
		).Handshake()
		if err == nil || err.Error() != "tls: MaxVersion prevents QUIC from using TLS 1.3" {
			t.Errorf("expected the handshake to fail")
		}
		if serverRecordLayer.alertSent != alertInternalError {
			t.Fatal("expected an internal error alert to be sent")
		}
	})
}

func TestForbiddenZeroRTT(t *testing.T) {
	// run the first handshake to get a session ticket
	clientConn, serverConn := localPipe(t)
	errChan := make(chan error, 1)
	go func() {
		tlsConn := Server(serverConn, testConfig.Clone(), nil)
		defer tlsConn.Close()
		err := tlsConn.Handshake()
		errChan <- err
		if err != nil {
			return
		}
		tlsConn.Write([]byte{0})
	}()

	clientConfig := testConfig.Clone()
	clientConfig.ClientSessionCache = NewLRUClientSessionCache(10)
	tlsConn := Client(clientConn, clientConfig, nil)
	if err := tlsConn.Handshake(); err != nil {
		t.Fatalf("first handshake failed: %s", err)
	}
	tlsConn.Read([]byte{0}) // make sure to read the session ticket
	tlsConn.Close()
	if err := <-errChan; err != nil {
		t.Fatalf("first handshake failed: %s", err)
	}

	sIn := make(chan []byte, 10)
	cIn := make(chan []byte, 10)
	cOut := make(chan []byte, 10)

	go func() {
		for {
			b, ok := <-cOut
			if !ok {
				return
			}
			if b[0] == typeClientHello {
				msg := &clientHelloMsg{}
				if ok := msg.unmarshal(b); !ok {
					panic("unmarshaling failed")
				}
				msg.earlyData = true
				msg.raw = nil
				b = msg.marshal()
			}
			sIn <- b
		}
	}()

	done := make(chan struct{})
	go func() {
		defer close(done)
		extraConf := &ExtraConfig{AlternativeRecordLayer: &recordLayer{in: cIn, out: cOut}}
		Client(&unusedConn{remoteAddr: clientConn.RemoteAddr()}, clientConfig, extraConf).Handshake()
	}()

	config := testConfig.Clone()
	config.MinVersion = VersionTLS13
	serverRecordLayer := &recordLayer{in: sIn, out: cIn}
	extraConf := &ExtraConfig{AlternativeRecordLayer: serverRecordLayer}
	tlsConn = Server(&unusedConn{}, config, extraConf)
	err := tlsConn.Handshake()
	if err == nil {
		t.Fatal("expected handshake to fail")
	}
	if err.Error() != "tls: client sent unexpected early data" {
		t.Fatalf("expected early data error")
	}
	if serverRecordLayer.alertSent != alertUnsupportedExtension {
		t.Fatal("expected an unsupported extension alert to be sent")
	}
	cIn <- []byte{0} // make the client handshake error
	<-done
}

func TestZeroRTTKeys(t *testing.T) {
	// run the first handshake to get a session ticket
	clientConn, serverConn := localPipe(t)
	errChan := make(chan error, 1)
	go func() {
		extraConf := &ExtraConfig{MaxEarlyData: 1000}
		tlsConn := Server(serverConn, testConfig, extraConf)
		defer tlsConn.Close()
		err := tlsConn.Handshake()
		errChan <- err
		if err != nil {
			return
		}
		tlsConn.Write([]byte{0})
	}()

	clientConfig := testConfig.Clone()
	clientConfig.ClientSessionCache = NewLRUClientSessionCache(10)
	tlsConn := Client(clientConn, clientConfig, nil)
	if err := tlsConn.Handshake(); err != nil {
		t.Fatalf("first handshake failed: %s", err)
	}
	tlsConn.Read([]byte{0}) // make sure to read the session ticket
	tlsConn.Close()
	if err := <-errChan; err != nil {
		t.Fatalf("first handshake failed: %s", err)
	}

	sIn := make(chan []byte, 10)
	sOut := make(chan interface{}, 10)
	defer close(sOut)
	cIn := make(chan []byte, 10)
	cOut := make(chan interface{}, 10)
	defer close(cOut)

	var serverEarlyData bool
	var serverExportedKey *exportedKey
	go func() {
		for {
			c, ok := <-sOut
			if !ok {
				return
			}
			if b, ok := c.([]byte); ok {
				if b[0] == typeEncryptedExtensions {
					var msg encryptedExtensionsMsg
					if ok := msg.unmarshal(b); !ok {
						panic("failed to unmarshal EncryptedExtensions")
					}
					serverEarlyData = msg.earlyData
				}
				cIn <- b
			}
			if k, ok := c.(*exportedKey); ok && k.encLevel == Encryption0RTT {
				serverExportedKey = k
			}
		}
	}()

	var clientEarlyData bool
	var clientExportedKey *exportedKey
	go func() {
		for {
			c, ok := <-cOut
			if !ok {
				return
			}
			if b, ok := c.([]byte); ok {
				if b[0] == typeClientHello {
					var msg clientHelloMsg
					if ok := msg.unmarshal(b); !ok {
						panic("failed to unmarshal ClientHello")
					}
					clientEarlyData = msg.earlyData
				}
				sIn <- b
			}
			if k, ok := c.(*exportedKey); ok && k.encLevel == Encryption0RTT {
				clientExportedKey = k
			}
		}
	}()

	errChan = make(chan error)
	go func() {
		extraConf := &ExtraConfig{
			AlternativeRecordLayer: &recordLayerWithKeys{in: sIn, out: sOut},
			MaxEarlyData:           1,
			Accept0RTT:             func([]byte) bool { return true },
		}
		tlsConn := Server(&unusedConn{}, testConfig, extraConf)
		defer tlsConn.Close()
		errChan <- tlsConn.Handshake()
	}()

	extraConf := &ExtraConfig{
		AlternativeRecordLayer: &recordLayerWithKeys{in: cIn, out: cOut},
		Enable0RTT:             true,
	}
	tlsConn = Client(&unusedConn{remoteAddr: clientConn.RemoteAddr()}, clientConfig, extraConf)
	defer tlsConn.Close()
	if err := tlsConn.Handshake(); err != nil {
		t.Fatalf("Handshake failed: %s", err)
	}
	if err := <-errChan; err != nil {
		t.Fatalf("Handshake failed: %s", err)
	}

	if !clientEarlyData {
		t.Fatal("expected the client to offer early data")
	}
	if !serverEarlyData {
		t.Fatal("expected the server to offer early data")
	}
	compareExportedKeys(t, clientExportedKey, serverExportedKey)
}

func TestEncodeIntoSessionTicket(t *testing.T) {
	raddr := &net.TCPAddr{IP: net.IPv4(127, 0, 0, 1), Port: 1234}
	sIn := make(chan []byte, 10)
	sOut := make(chan []byte, 10)

	// do a first handshake and encode a "foobar" into the session ticket
	errChan := make(chan error, 1)
	stChan := make(chan []byte, 1)
	go func() {
		extraConf := &ExtraConfig{
			AlternativeRecordLayer: &recordLayer{in: sIn, out: sOut},
			MaxEarlyData:           1,
		}
		server := Server(&unusedConn{remoteAddr: raddr}, testConfig, extraConf)
		defer server.Close()
		err := server.Handshake()
		if err != nil {
			errChan <- err
			return
		}
		st, err := server.GetSessionTicket([]byte("foobar"))
		if err != nil {
			errChan <- err
			return
		}
		stChan <- st
		errChan <- nil
	}()

	clientConf := testConfig.Clone()
	clientConf.ClientSessionCache = NewLRUClientSessionCache(10)
	extraConf := &ExtraConfig{AlternativeRecordLayer: &recordLayer{in: sOut, out: sIn}}
	client := Client(&unusedConn{remoteAddr: raddr}, clientConf, extraConf)
	if err := client.Handshake(); err != nil {
		t.Fatalf("first handshake failed %s", err)
	}
	if err := <-errChan; err != nil {
		t.Fatalf("first handshake failed %s", err)
	}
	sOut <- <-stChan
	if err := client.HandlePostHandshakeMessage(); err != nil {
		t.Fatalf("handling the session ticket failed: %s", err)
	}
	client.Close()

	dataChan := make(chan []byte, 1)
	errChan = make(chan error, 1)
	go func() {
		extraConf := &ExtraConfig{
			AlternativeRecordLayer: &recordLayer{in: sIn, out: sOut},
			MaxEarlyData:           1,
			Accept0RTT: func(data []byte) bool {
				dataChan <- data
				return true
			},
		}
		server := Server(&unusedConn{remoteAddr: raddr}, testConfig, extraConf)
		defer server.Close()
		errChan <- server.Handshake()
	}()

	extraConf2 := extraConf.Clone()
	extraConf2.Enable0RTT = true
	client = Client(&unusedConn{remoteAddr: raddr}, clientConf, extraConf2)
	if err := client.Handshake(); err != nil {
		t.Fatalf("second handshake failed %s", err)
	}
	defer client.Close()
	if err := <-errChan; err != nil {
		t.Fatalf("second handshake failed %s", err)
	}
	if len(dataChan) != 1 {
		t.Fatal("expected to receive application data")
	}
	if data := <-dataChan; !bytes.Equal(data, []byte("foobar")) {
		t.Fatalf("expected to receive a foobar, got %s", string(data))
	}
}

func TestZeroRTTRejection(t *testing.T) {
	for _, doReject := range []bool{true, false} {
		t.Run(fmt.Sprintf("doing reject: %t", doReject), func(t *testing.T) {
			raddr := &net.TCPAddr{IP: net.IPv4(127, 0, 0, 1), Port: 1234}
			sIn := make(chan []byte, 10)
			sOut := make(chan []byte, 10)

			// do a first handshake and encode a "foobar" into the session ticket
			errChan := make(chan error, 1)
			go func() {
				extraConf := &ExtraConfig{
					AlternativeRecordLayer: &recordLayer{in: sIn, out: sOut},
					MaxEarlyData:           1,
				}
				server := Server(&unusedConn{remoteAddr: raddr}, testConfig, extraConf)
				defer server.Close()
				err := server.Handshake()
				if err != nil {
					errChan <- err
					return
				}
				st, err := server.GetSessionTicket(nil)
				if err != nil {
					errChan <- err
					return
				}
				sOut <- st
				errChan <- nil
			}()

			conf := testConfig.Clone()
			conf.ClientSessionCache = NewLRUClientSessionCache(10)
			extraConf := &ExtraConfig{AlternativeRecordLayer: &recordLayer{in: sOut, out: sIn}}
			client := Client(&unusedConn{remoteAddr: raddr}, conf, extraConf)
			if err := client.Handshake(); err != nil {
				t.Fatalf("first handshake failed %s", err)
			}
			if err := <-errChan; err != nil {
				t.Fatalf("first handshake failed %s", err)
			}
			if err := client.HandlePostHandshakeMessage(); err != nil {
				t.Fatalf("handling the session ticket failed: %s", err)
			}
			client.Close()

			// now dial the second connection
			errChan = make(chan error, 1)
			connStateChan := make(chan ConnectionStateWith0RTT, 1)
			go func() {
				extraConf := &ExtraConfig{
					AlternativeRecordLayer: &recordLayer{in: sIn, out: sOut},
					MaxEarlyData:           1,
					Accept0RTT:             func(data []byte) bool { return !doReject },
				}
				server := Server(&unusedConn{remoteAddr: raddr}, testConfig, extraConf)
				defer server.Close()
				errChan <- server.Handshake()
				connStateChan <- server.ConnectionStateWith0RTT()
			}()

			extraConf2 := extraConf.Clone()
			extraConf2.Enable0RTT = true
			var rejected bool
			extraConf2.Rejected0RTT = func() { rejected = true }
			client = Client(&unusedConn{remoteAddr: raddr}, conf, extraConf2)
			if err := client.Handshake(); err != nil {
				t.Fatalf("second handshake failed %s", err)
			}
			defer client.Close()
			if err := <-errChan; err != nil {
				t.Fatalf("second handshake failed %s", err)
			}
			if rejected != doReject {
				t.Fatal("wrong rejection")
			}
			if client.ConnectionStateWith0RTT().Used0RTT == doReject {
				t.Fatal("wrong connection state on the client")
			}
			if (<-connStateChan).Used0RTT == doReject {
				t.Fatal("wrong connection state on the server")
			}
		})
	}
}

func TestZeroRTTALPN(t *testing.T) {
	run := func(t *testing.T, proto1, proto2 string, expectReject bool) {
		raddr := &net.TCPAddr{IP: net.IPv4(127, 0, 0, 1), Port: 1234}
		sIn := make(chan []byte, 10)
		sOut := make(chan []byte, 10)

		// do a first handshake and encode a "foobar" into the session ticket
		errChan := make(chan error, 1)
		go func() {
			serverConf := testConfig.Clone()
			serverConf.NextProtos = []string{proto1}
			extraConf := &ExtraConfig{
				AlternativeRecordLayer: &recordLayer{in: sIn, out: sOut},
				MaxEarlyData:           1,
			}
			server := Server(&unusedConn{remoteAddr: raddr}, serverConf, extraConf)
			defer server.Close()
			err := server.Handshake()
			if err != nil {
				errChan <- err
				return
			}
			st, err := server.GetSessionTicket(nil)
			if err != nil {
				errChan <- err
				return
			}
			sOut <- st
			errChan <- nil
		}()

		clientConf := testConfig.Clone()
		clientConf.NextProtos = []string{proto1}
		clientConf.ClientSessionCache = NewLRUClientSessionCache(10)
		extraConf := &ExtraConfig{AlternativeRecordLayer: &recordLayer{in: sOut, out: sIn}}
		client := Client(&unusedConn{remoteAddr: raddr}, clientConf, extraConf)
		if err := client.Handshake(); err != nil {
			t.Fatalf("first handshake failed %s", err)
		}
		if err := <-errChan; err != nil {
			t.Fatalf("first handshake failed %s", err)
		}
		if err := client.HandlePostHandshakeMessage(); err != nil {
			t.Fatalf("handling the session ticket failed: %s", err)
		}
		client.Close()

		// now dial the second connection
		errChan = make(chan error, 1)
		connStateChan := make(chan ConnectionStateWith0RTT, 1)
		go func() {
			serverConf := testConfig.Clone()
			serverConf.NextProtos = []string{proto2}
			extraConf := &ExtraConfig{
				AlternativeRecordLayer: &recordLayer{in: sIn, out: sOut},
				Accept0RTT:             func([]byte) bool { return true },
				MaxEarlyData:           1,
			}
			server := Server(&unusedConn{remoteAddr: raddr}, serverConf, extraConf)
			defer server.Close()
			errChan <- server.Handshake()
			connStateChan <- server.ConnectionStateWith0RTT()
		}()

		clientConf.NextProtos = []string{proto2}
		extraConf.Enable0RTT = true
		var rejected bool
		extraConf.Rejected0RTT = func() { rejected = true }
		client = Client(&unusedConn{remoteAddr: raddr}, clientConf, extraConf)
		if err := client.Handshake(); err != nil {
			t.Fatalf("second handshake failed %s", err)
		}
		defer client.Close()
		if err := <-errChan; err != nil {
			t.Fatalf("second handshake failed %s", err)
		}
		if expectReject {
			if !rejected {
				t.Fatal("expected 0-RTT to be rejected")
			}
			if client.ConnectionStateWith0RTT().Used0RTT {
				t.Fatal("expected 0-RTT to be rejected")
			}
			if (<-connStateChan).Used0RTT {
				t.Fatal("expected 0-RTT to be rejected")
			}
		} else {
			if rejected {
				t.Fatal("didn't expect 0-RTT to be rejected")
			}
			if !client.ConnectionStateWith0RTT().Used0RTT {
				t.Fatal("didn't expect 0-RTT to be rejected")
			}
			if !(<-connStateChan).Used0RTT {
				t.Fatal("didn't expect 0-RTT to be rejected")
			}
		}
	}

	t.Run("with the same alpn", func(t *testing.T) {
		run(t, "proto1", "proto1", false)
	})
	t.Run("with different alpn", func(t *testing.T) {
		run(t, "proto1", "proto2", true)
	})
}
