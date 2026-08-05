/*
 * Override the signed-division runtime helpers to prevent linking libgcc's full __divsi3
 * (handles INT_MIN/-1 overflow specially; not needed by anything in this firmware). This
 * implementation splits sign from magnitude and calls the unsigned divide (__aeabi_uidiv /
 * __udivsi3), which is already linked for other calculations -- so this costs only the few
 * sign-handling instructions below instead of the ~460 bytes of libgcc's routine. Divide-by-
 * zero still traps via __aeabi_idiv0, since that's reached through the unsigned divide too.
 * Saving flash.
 * 
 * This file must be compiled without -flto (link-time optimization) to work.
 */
int __aeabi_idiv(int a, int b)
{
    unsigned int ua = (a < 0) ? (unsigned int)(-a) : (unsigned int)a;
    unsigned int ub = (b < 0) ? (unsigned int)(-b) : (unsigned int)b;
    unsigned int q = ua / ub;
    return ((a < 0) != (b < 0)) ? -(int)q : (int)q;
}

int __divsi3(int a, int b)
{
    return __aeabi_idiv(a, b);
}
