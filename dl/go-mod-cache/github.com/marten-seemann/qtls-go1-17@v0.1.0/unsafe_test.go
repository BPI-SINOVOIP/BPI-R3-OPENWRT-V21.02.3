package qtls

import (
	"bytes"
	"crypto/x509"
	"errors"
	"fmt"
	"reflect"
	"testing"
	"time"
)

func TestUnsafeConversionIsSafe(t *testing.T) {
	type target struct {
		Name    string
		Version string

		callback func(label string, length int) error
	}

	type renamedField struct {
		NewName string
		Version string

		callback func(label string, length int) error
	}

	type renamedPrivateField struct {
		Name    string
		Version string

		cb func(label string, length int) error
	}

	type additionalField struct {
		Name    string
		Version string

		callback func(label string, length int) error
		secret   []byte
	}

	type interchangedFields struct {
		Version string
		Name    string

		callback func(label string, length int) error
	}

	type renamedCallbackFunctionParams struct { // should be equivalent
		Name    string
		Version string

		callback func(newLabel string, length int) error
	}

	testCases := []struct {
		name string
		from interface{}
		to   interface{}
		safe bool
	}{
		{"same struct", &target{}, &target{}, true},
		{"struct with a renamed field", &target{}, &renamedField{}, false},
		{"struct with a renamed private field", &target{}, &renamedPrivateField{}, false},
		{"struct with an additional field", &target{}, &additionalField{}, false},
		{"struct with interchanged fields", &target{}, &interchangedFields{}, false},
		{"struct with a renamed callback parameter", &target{}, &renamedCallbackFunctionParams{}, true},
	}

	for _, testCase := range testCases {
		t.Run(fmt.Sprintf("unsafe conversion: %s", testCase.name), func(t *testing.T) {
			if structsEqual(testCase.from, testCase.to) != testCase.safe {
				t.Errorf("invalid unsafe conversion")
			}
		})
	}
}

func TestConnectionStateReinterpretCast(t *testing.T) {
	var ekmLabel string
	var ekmContext []byte
	var ekmLength int
	state := connectionState{
		Version:            1234,
		HandshakeComplete:  true,
		DidResume:          true,
		CipherSuite:        4321,
		NegotiatedProtocol: "foobar",
		ServerName:         "server",
		PeerCertificates:   []*x509.Certificate{{Raw: []byte("foobar")}},
		OCSPResponse:       []byte("foo"),
		TLSUnique:          []byte("bar"),
		ekm: func(label string, context []byte, length int) ([]byte, error) {
			ekmLabel = label
			ekmContext = append(ekmContext, context...)
			ekmLength = length
			return []byte("ekm"), errors.New("ekm error")
		},
	}
	tlsState := toConnectionState(state)
	if tlsState.Version != 1234 {
		t.Error("Version doesn't match")
	}
	if !tlsState.HandshakeComplete {
		t.Error("HandshakeComplete doesn't match")
	}
	if !tlsState.DidResume {
		t.Error("DidResume doesn't match")
	}
	if tlsState.CipherSuite != 4321 {
		t.Error("CipherSuite doesn't match")
	}
	if tlsState.NegotiatedProtocol != "foobar" {
		t.Error("NegotiatedProtocol doesn't match")
	}
	if tlsState.ServerName != "server" {
		t.Error("ServerName doesn't match")
	}
	if len(tlsState.PeerCertificates) != 1 || !bytes.Equal(tlsState.PeerCertificates[0].Raw, []byte("foobar")) {
		t.Error("PeerCertificates don't match")
	}
	if !bytes.Equal(tlsState.OCSPResponse, []byte("foo")) {
		t.Error("OSCPResponse doesn't match")
	}
	if !bytes.Equal(tlsState.TLSUnique, []byte("bar")) {
		t.Error("TLSUnique doesn't match")
	}

	key, err := tlsState.ExportKeyingMaterial("label", []byte("context"), 42)
	if !bytes.Equal(key, []byte("ekm")) {
		t.Error("exported key doesn't match")
	}
	if err == nil || err.Error() != "ekm error" {
		t.Error("key export error doesn't match")
	}
	if ekmLabel != "label" {
		t.Error("key export label doesn't match")
	}
	if !bytes.Equal(ekmContext, []byte("context")) {
		t.Error("key export context doesn't match")
	}
	if ekmLength != 42 {
		t.Error("key export length doesn't match")
	}
}

func TestClientSessionStateReinterpretCast(t *testing.T) {
	state := &clientSessionState{
		sessionTicket: []byte("foobar"),
		receivedAt:    time.Now(),
		nonce:         []byte("foo"),
		useBy:         time.Now().Add(time.Hour),
		ageAdd:        1234,
	}
	if !reflect.DeepEqual(fromClientSessionState(toClientSessionState(state)), state) {
		t.Fatal("failed")
	}
}

// func TestClientSessionStateReinterpretCast(t *testing.T) {
// 	state := &clientSessionState{
// 		sessionTicket: []byte("foobar"),
// 		receivedAt:    time.Now(),
// 		nonce:         []byte("foo"),
// 		useBy:         time.Now().Add(time.Hour),
// 		ageAdd:        1234,
// 	}
// 	if !reflect.DeepEqual(fromClientSessionState(toClientSessionState(state)), state) {
// 		t.Fatal("failed")
// 	}
// }
