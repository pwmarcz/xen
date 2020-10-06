#ifndef __X86_DIV64
#define __X86_DIV64

#include <xen/types.h>

#define do_div(n,base) ({                       \
    uint32_t __base = (base);                   \
    uint32_t __rem;                             \
    __rem = ((uint64_t)(n)) % __base;           \
    (n) = ((uint64_t)(n)) / __base;             \
    __rem;                                      \
})

/*
 * div_u64_rem - unsigned 64bit divide with 32bit divisor
 * @dividend:  64bit dividend
 * @divisor:   32bit divisor
 * @remainder: 32bit remainder
 */
static inline uint64_t div_u64_rem(uint64_t dividend, uint32_t divisor,
                                   uint32_t *remainder)
{
    *remainder = do_div(dividend, divisor);
    return dividend;
}

static inline uint64_t div_u64(uint64_t dividend, uint32_t  divisor)
{
    uint32_t remainder;

    return div_u64_rem(dividend, divisor, &remainder);
}

/*
 * div64_u64 - unsigned 64bit divide with 64bit divisor
 * @dividend: 64bit dividend
 * @divisor:  64bit divisor
 *
 * This implementation is a modified version of the algorithm proposed
 * by the book 'Hacker's Delight'.  The original source and full proof
 * can be found here and is available for use without restriction.
 *
 * 'http://www.hackersdelight.org/HDcode/newCode/divDouble.c.txt'
 */
static inline uint64_t div64_u64(uint64_t dividend, uint64_t divisor)
{
    uint32_t high = divisor >> 32;
    uint64_t quot;

    if ( high == 0 )
        quot = div_u64(dividend, divisor);
    else
    {
        int n = 1 + fls(high);

        quot = div_u64(dividend >> n, divisor >> n);

        if ( quot != 0 )
            quot--;
        if ( (dividend - quot * divisor) >= divisor )
            quot++;
    }
    return quot;
}

/*
 * div_u64_rem - signed 64bit divide with 32bit divisor
 * @dividend:  64bit dividend
 * @divisor:   32bit divisor
 * @remainder: 32bit remainder
 */
static inline int64_t div_s64_rem(int64_t dividend, int32_t divisor,
                                  int32_t *remainder)
{
    int64_t quotient;

    if ( dividend < 0 )
    {
        quotient = div_u64_rem(-dividend, ABS(divisor),
                               (uint32_t *)remainder);
        *remainder = -*remainder;
        if ( divisor > 0 )
            quotient = -quotient;
    }
    else
    {
        quotient = div_u64_rem(dividend, ABS(divisor),
                        (uint32_t *)remainder);
        if ( divisor < 0 )
            quotient = -quotient;
    }
    return quotient;
}

/*
 * div_s64 - signed 64bit divide with 32bit divisor
 */
static inline int64_t div_s64(int64_t dividend, int32_t divisor)
{
    int32_t remainder;

    return div_s64_rem(dividend, divisor, &remainder);
}

#endif
