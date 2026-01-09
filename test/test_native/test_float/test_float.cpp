#include <array>
#include <cmath>
#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

union fi { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
    float f;
    uint32_t i;
};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,cppcoreguidelines-pro-type-union-access,hicpp-use-auto,modernize-use-auto)
void test_float()
{
    fi v {};
    uint32_t mantissa = 0;
    uint32_t exponent = 0;

    TEST_ASSERT_EQUAL_FLOAT(0.0F, v.f);
    TEST_ASSERT_EQUAL(0, v.i & 0x80000000U); // sign bit
    mantissa =  v.i & 0x07fffffU;
    exponent = (v.i & 0x7f800000U) >> 23U;
    TEST_ASSERT_EQUAL(0, mantissa);
    TEST_ASSERT_EQUAL(0, exponent);

    v.f = 1.0F;
    TEST_ASSERT_EQUAL_FLOAT(1.0F, v.f);
    TEST_ASSERT_EQUAL(0, v.i & 0x80000000U); // sign bit
    mantissa =  v.i & 0x07fffffU;
    exponent = (v.i & 0x7f800000U) >> 23U;
    TEST_ASSERT_EQUAL(0, mantissa);
    TEST_ASSERT_EQUAL(127, exponent); // twos complement

    v.f = 2.0F;
    TEST_ASSERT_EQUAL_FLOAT(2.0F, v.f);
    TEST_ASSERT_EQUAL(0, v.i & 0x80000000); // sign bit
    mantissa =  v.i & 0x07fffffU;
    exponent = (v.i & 0x7f800000U) >> 23U;
    TEST_ASSERT_EQUAL(0, mantissa);
    TEST_ASSERT_EQUAL(128, exponent);

    v.f = 0.5F;
    TEST_ASSERT_EQUAL_FLOAT(0.5F, v.f);
    TEST_ASSERT_EQUAL(0, v.i & 0x80000000U); // sign bit
    mantissa =  v.i & 0x07fffffU;
    exponent = (v.i & 0x7f800000U) >> 23U;
    TEST_ASSERT_EQUAL(0, mantissa);
    TEST_ASSERT_EQUAL(126, exponent);

    v.f = 0.75F;
    TEST_ASSERT_EQUAL_FLOAT(0.75F, v.f);
    TEST_ASSERT_EQUAL(0, v.i & 0x80000000); // sign bit
    mantissa =  v.i & 0x07fffffU;
    exponent = (v.i & 0x7f800000U) >> 23U;
    TEST_ASSERT_EQUAL(0x400000, mantissa);
    TEST_ASSERT_EQUAL(126, exponent);
}

int32_t float32ToInt24(float x)
{
    const union { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        float f;
        uint32_t i;
    } n {.f = x};

    const auto sign     = static_cast<uint8_t>((n.i >> 31U) & 0x1U); // 0x1000 0000
    const auto exponent = static_cast<uint8_t>((n.i >> 23U) & 0xFFU); // 0x7F80 0000
    if (exponent == 0) {
        return 0;
    }

    const uint32_t mantissa = (n.i & 0x7FFFFFU) | 0x800000U; // 0x007F FFFF

    const auto i = static_cast<int32_t>(mantissa >> (22U - (exponent - 0x80U)));
    return sign ? -i : i;
}

void test_float_convert()
{
    TEST_ASSERT_EQUAL(1, float32ToInt24(1.0F));
    TEST_ASSERT_EQUAL(0, float32ToInt24(0.5F));
    TEST_ASSERT_EQUAL(1, float32ToInt24(1.5F));
    TEST_ASSERT_EQUAL(2, float32ToInt24(2.0F));
    TEST_ASSERT_EQUAL(19, float32ToInt24(19.0F));
    TEST_ASSERT_EQUAL(229, float32ToInt24(229.0F));
    TEST_ASSERT_EQUAL(-119, float32ToInt24(-119.0F));
}

int32_t float32_to_Q12dot4(float x)
{
    const union { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        float f;
        uint32_t i;
    } n {.f = x};

    const auto sign     = static_cast<uint8_t>((n.i >> 31U) & 0x1U);
    const auto exponent = static_cast<uint8_t>((n.i >> 23U) & 0xFFU);
    if (exponent == 0) {
        return 0;
    }

    const uint32_t mantissa = (n.i & 0x7FFFFFU) | 0x800000U; // 0x007F FFFF, or in implicit bit

    const auto i = static_cast<int32_t>(mantissa >> ((22U-11U) - (exponent - 0x80)));
    return sign ? -i : i;
}

int32_t ubyte4float_to_Q12dot4(const uint8_t f[4]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    /*union {
        uint8_t b[4];
        uint32_t i;
    } n;
    n.b[0] = f[0];
    n.b[1] = f[1];
    n.b[2] = f[2];
    n.b[3] = f[3];*/
    union bi_t { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        std::array<uint8_t, 4> b;
        uint32_t i;
    };
    const bi_t n = { .b = { f[0], f[1], f[2], f[3] } }; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    const uint8_t exponent = static_cast<uint8_t>((n.i >> 23U) & 0xFFU); // 0x7F80 0000
    if (exponent == 0) {
        return 0;
    }

    const uint8_t sign     = static_cast<uint8_t>((n.i >> 31U) & 0x1U); // 0x1000 0000
    const uint32_t mantissa = (n.i & 0x7FFFFFU) | 0x800000U; // 0x007F FFFF, OR in implicit bit

    const int32_t i = static_cast<int32_t>(mantissa >> ((22U-11U) - (exponent - 0x80U))); // -Wshift-count-overflow
    return sign ? -i : i;
}

float Q12dot4_to_float32(int16_t a)
{
    return static_cast<float>(a) / 2048.0F;
}

void test_fixed_convert()
{
    TEST_ASSERT_EQUAL(4096, float32_to_Q12dot4(2.0F));
    TEST_ASSERT_EQUAL(-4096, float32_to_Q12dot4(-2.0F));

    const float x = 0.48F;
    const auto a = static_cast<int16_t>(float32_to_Q12dot4(x));
    TEST_ASSERT_EQUAL(floor(0.48*2048), a);
    const float y = Q12dot4_to_float32(a);
    TEST_ASSERT_FLOAT_WITHIN(0.00008F*x, x, y);

    TEST_ASSERT_EQUAL(2048, float32_to_Q12dot4(1.0F));
    TEST_ASSERT_EQUAL(-2048, float32_to_Q12dot4(-1.0F));

    TEST_ASSERT_EQUAL(1024, float32_to_Q12dot4(0.5F));
    TEST_ASSERT_EQUAL(-1024, float32_to_Q12dot4(-0.5F));

    TEST_ASSERT_EQUAL(512, float32_to_Q12dot4(0.25F));
    TEST_ASSERT_EQUAL(-512, float32_to_Q12dot4(-0.25F));

    TEST_ASSERT_EQUAL(4096, float32_to_Q12dot4(2.0F));
    TEST_ASSERT_EQUAL(8192, float32_to_Q12dot4(4.0F));
    TEST_ASSERT_EQUAL(16384, float32_to_Q12dot4(8.0F));
    TEST_ASSERT_EQUAL(32768, float32_to_Q12dot4(16.0F));

}

void test_byte_convert()
{
    union bf { // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
        std::array<uint8_t, 4> b;
        float f;
    };

    bf x { .f = 2.0F};
    TEST_ASSERT_EQUAL_FLOAT(2.0F, x.f);


    x.f = 0.48F; // NOLINT(cppcoreguidelines-pro-type-union-access)
    const auto a = static_cast<int16_t>(ubyte4float_to_Q12dot4(&x.b[0]));
    TEST_ASSERT_EQUAL(floor(0.48*2048), a);

    const float y = Q12dot4_to_float32(a);
    TEST_ASSERT_FLOAT_WITHIN(0.00008F*x.f, x.f, y);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,cppcoreguidelines-pro-type-union-access,hicpp-use-auto,modernize-use-auto)


int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_float);
    RUN_TEST(test_float_convert);
    RUN_TEST(test_fixed_convert);

    UNITY_END();
}
