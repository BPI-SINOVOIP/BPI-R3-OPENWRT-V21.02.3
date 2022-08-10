package ss_bloomring

import (
	"fmt"
	"os"
	"testing"
)

var (
	bloomRingInstance *BloomRing
)

const (
	DefaultSFCapacity = 1e6
	// FalsePositiveRate
	DefaultSFFPR  = 1e-6
	DefaultSFSlot = 10
)

func TestMain(m *testing.M) {
	bloomRingInstance = NewBloomRing(DefaultSFSlot, int(DefaultSFCapacity),
		DefaultSFFPR)
	os.Exit(m.Run())
}

func TestBloomRing_Add(t *testing.T) {
	defer func() {
		if any := recover(); any != nil {
			t.Fatalf("Should not got panic while adding item: %v", any)
		}
	}()
	bloomRingInstance.Add(make([]byte, 16))
}

func TestBloomRing_NilAdd(t *testing.T) {
	defer func() {
		if any := recover(); any != nil {
			t.Fatalf("Should not got panic while adding item: %v", any)
		}
	}()
	var nilRing *BloomRing
	nilRing.Add(make([]byte, 16))
}

func TestBloomRing_Test(t *testing.T) {
	buf := []byte("shadowsocks")
	bloomRingInstance.Add(buf)
	if !bloomRingInstance.Test(buf) {
		t.Fatal("Test on filter missing")
	}
}

func TestBloomRing_NilTestIsFalse(t *testing.T) {
	var nilRing *BloomRing
	if nilRing.Test([]byte("shadowsocks")) {
		t.Fatal("Test should return false for nil BloomRing")
	}
}

func BenchmarkBloomRing(b *testing.B) {
	// Generate test samples with different length
	samples := make([][]byte, DefaultSFCapacity-DefaultSFSlot)
	var checkPoints [][]byte
	for i := 0; i < len(samples); i++ {
		samples[i] = []byte(fmt.Sprint(i))
		if i%1000 == 0 {
			checkPoints = append(checkPoints, samples[i])
		}
	}
	b.Logf("Generated %d samples and %d check points", len(samples), len(checkPoints))
	for i := 1; i < 16; i++ {
		b.Run(fmt.Sprintf("Slot%d", i), benchmarkBloomRing(samples, checkPoints, i))
	}
}

func benchmarkBloomRing(samples, checkPoints [][]byte, slot int) func(*testing.B) {
	filter := NewBloomRing(slot, int(DefaultSFCapacity), DefaultSFFPR)
	for _, sample := range samples {
		filter.Add(sample)
	}
	return func(b *testing.B) {
		b.ResetTimer()
		b.ReportAllocs()
		for i := 0; i < b.N; i++ {
			for _, cp := range checkPoints {
				filter.Test(cp)
			}
		}
	}
}
