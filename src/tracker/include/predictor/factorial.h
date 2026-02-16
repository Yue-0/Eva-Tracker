/* @author: YueLin */

#ifndef Factorial

#define Factorial int
// #define Factorial unsigned long long

static const Factorial __factorial[] = {
      1
    , 1
    , 2
    , 6
    , 24
    , 120
    , 720
    , 5040
    , 40320
    , 362880
    , 3628800
    , 39916800
    , 479001600

    /* If you need larger factorials, 
       please comment line 5,
       and uncomment line 6 and the following lines */

    // , 6227020800
    // , 87178291200
    // , 1307674368000
    // , 20922789888000
    // , 355687428096000
    // , 6402373705728000
    // , 121645100408832000
    // , 2432902008176640000
    // , 51090942171709440000
    // , 1124000727777607680000
    // , 25852016738884976640000
    // , 620448401733239439360000
    // , 15511210043330985984000000
    // , 403291461126605635584000000
    // , 10888869450418352160768000000
    // , 304888344611713860501504000000
    // , 8841761993739701954543616000000
    // , 265252859812191058636308480000000
    // , 8222838654177922817725562880000000
    // , 263130836933693530167218012160000000
    // , 8683317618811886495518194401280000000
};

inline Factorial factorial(int n) 
{
    return __factorial[n];
}

inline Factorial factorial(int n, int m)
{
    return factorial(n) / factorial(m);
}

inline Factorial combination(int n, int m)
{
    if(m < 0 || m > n) return 0;
    return factorial(n, m) / factorial(n - m);
}

#endif
