/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) mapping_f1_InkASXGEZkQWNKKgVChq_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fmax CASADI_PREFIX(fmax)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

casadi_real casadi_fmax(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>y ? x : y;
#else
  return fmax(x, y);
#endif
}

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* mapping_f1_InkASXGEZkQWNKKgVChq:(i0[10],i1[14])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=0.;
  a1=arg[1]? arg[1][8] : 0;
  a2=arg[1]? arg[1][10] : 0;
  a3=cos(a2);
  a4=arg[1]? arg[1][11] : 0;
  a3=(a3*a4);
  a1=(a1+a3);
  a3=arg[1]? arg[1][5] : 0;
  a5=(a1-a3);
  a5=casadi_sq(a5);
  a6=arg[1]? arg[1][9] : 0;
  a7=sin(a2);
  a7=(a7*a4);
  a6=(a6+a7);
  a7=arg[1]? arg[1][6] : 0;
  a8=(a6-a7);
  a8=casadi_sq(a8);
  a5=(a5+a8);
  a5=sqrt(a5);
  a8=arg[1]? arg[1][0] : 0;
  a9=arg[1]? arg[1][2] : 0;
  a10=cos(a9);
  a11=arg[0]? arg[0][0] : 0;
  a10=(a10*a11);
  a8=(a8+a10);
  a10=(a8-a3);
  a10=casadi_sq(a10);
  a12=arg[1]? arg[1][1] : 0;
  a13=sin(a9);
  a13=(a13*a11);
  a12=(a12+a13);
  a13=(a12-a7);
  a13=casadi_sq(a13);
  a10=(a10+a13);
  a10=sqrt(a10);
  a5=(a5-a10);
  a5=(-a5);
  a5=casadi_fmax(a0,a5);
  a10=arg[1]? arg[1][13] : 0;
  a13=casadi_sq(a10);
  a14=arg[0]? arg[0][1] : 0;
  a9=(a9+a14);
  a14=cos(a9);
  a14=(a14*a11);
  a14=(a14-a4);
  a14=casadi_sq(a14);
  a15=sin(a9);
  a15=(a15*a11);
  a15=(a15-a4);
  a15=casadi_sq(a15);
  a14=(a14+a15);
  a13=(a13*a14);
  a15=(a8-a1);
  a15=casadi_sq(a15);
  a16=(a12-a6);
  a16=casadi_sq(a16);
  a15=(a15+a16);
  a15=(a15*a14);
  a14=cos(a9);
  a14=(a14*a11);
  a14=(a14-a4);
  a16=(a8-a1);
  a14=(a14*a16);
  a16=sin(a9);
  a16=(a16*a11);
  a16=(a16-a4);
  a11=(a12-a6);
  a16=(a16*a11);
  a14=(a14+a16);
  a14=casadi_sq(a14);
  a15=(a15-a14);
  a13=(a13-a15);
  a13=casadi_fmax(a0,a13);
  a5=(a5*a13);
  a13=arg[1]? arg[1][12] : 0;
  a2=(a2+a13);
  a15=cos(a2);
  a15=(a15*a4);
  a1=(a1+a15);
  a15=(a1-a3);
  a15=casadi_sq(a15);
  a14=sin(a2);
  a14=(a14*a4);
  a6=(a6+a14);
  a14=(a6-a7);
  a14=casadi_sq(a14);
  a15=(a15+a14);
  a15=sqrt(a15);
  a14=cos(a9);
  a16=arg[0]? arg[0][2] : 0;
  a14=(a14*a16);
  a8=(a8+a14);
  a14=(a8-a3);
  a14=casadi_sq(a14);
  a11=sin(a9);
  a11=(a11*a16);
  a12=(a12+a11);
  a11=(a12-a7);
  a11=casadi_sq(a11);
  a14=(a14+a11);
  a14=sqrt(a14);
  a15=(a15-a14);
  a15=(-a15);
  a15=casadi_fmax(a0,a15);
  a14=casadi_sq(a10);
  a11=arg[0]? arg[0][3] : 0;
  a9=(a9+a11);
  a11=cos(a9);
  a11=(a11*a16);
  a11=(a11-a4);
  a11=casadi_sq(a11);
  a17=sin(a9);
  a17=(a17*a16);
  a17=(a17-a4);
  a17=casadi_sq(a17);
  a11=(a11+a17);
  a14=(a14*a11);
  a17=(a8-a1);
  a17=casadi_sq(a17);
  a18=(a12-a6);
  a18=casadi_sq(a18);
  a17=(a17+a18);
  a17=(a17*a11);
  a11=cos(a9);
  a11=(a11*a16);
  a11=(a11-a4);
  a18=(a8-a1);
  a11=(a11*a18);
  a18=sin(a9);
  a18=(a18*a16);
  a18=(a18-a4);
  a16=(a12-a6);
  a18=(a18*a16);
  a11=(a11+a18);
  a11=casadi_sq(a11);
  a17=(a17-a11);
  a14=(a14-a17);
  a14=casadi_fmax(a0,a14);
  a15=(a15*a14);
  a5=(a5+a15);
  a2=(a2+a13);
  a15=cos(a2);
  a15=(a15*a4);
  a1=(a1+a15);
  a15=(a1-a3);
  a15=casadi_sq(a15);
  a14=sin(a2);
  a14=(a14*a4);
  a6=(a6+a14);
  a14=(a6-a7);
  a14=casadi_sq(a14);
  a15=(a15+a14);
  a15=sqrt(a15);
  a14=cos(a9);
  a17=arg[0]? arg[0][4] : 0;
  a14=(a14*a17);
  a8=(a8+a14);
  a14=(a8-a3);
  a14=casadi_sq(a14);
  a11=sin(a9);
  a11=(a11*a17);
  a12=(a12+a11);
  a11=(a12-a7);
  a11=casadi_sq(a11);
  a14=(a14+a11);
  a14=sqrt(a14);
  a15=(a15-a14);
  a15=(-a15);
  a15=casadi_fmax(a0,a15);
  a14=casadi_sq(a10);
  a11=arg[0]? arg[0][5] : 0;
  a9=(a9+a11);
  a11=cos(a9);
  a11=(a11*a17);
  a11=(a11-a4);
  a11=casadi_sq(a11);
  a18=sin(a9);
  a18=(a18*a17);
  a18=(a18-a4);
  a18=casadi_sq(a18);
  a11=(a11+a18);
  a14=(a14*a11);
  a18=(a8-a1);
  a18=casadi_sq(a18);
  a16=(a12-a6);
  a16=casadi_sq(a16);
  a18=(a18+a16);
  a18=(a18*a11);
  a11=cos(a9);
  a11=(a11*a17);
  a11=(a11-a4);
  a16=(a8-a1);
  a11=(a11*a16);
  a16=sin(a9);
  a16=(a16*a17);
  a16=(a16-a4);
  a17=(a12-a6);
  a16=(a16*a17);
  a11=(a11+a16);
  a11=casadi_sq(a11);
  a18=(a18-a11);
  a14=(a14-a18);
  a14=casadi_fmax(a0,a14);
  a15=(a15*a14);
  a5=(a5+a15);
  a2=(a2+a13);
  a15=cos(a2);
  a15=(a15*a4);
  a1=(a1+a15);
  a15=(a1-a3);
  a15=casadi_sq(a15);
  a14=sin(a2);
  a14=(a14*a4);
  a6=(a6+a14);
  a14=(a6-a7);
  a14=casadi_sq(a14);
  a15=(a15+a14);
  a15=sqrt(a15);
  a14=cos(a9);
  a18=arg[0]? arg[0][6] : 0;
  a14=(a14*a18);
  a8=(a8+a14);
  a14=(a8-a3);
  a14=casadi_sq(a14);
  a11=sin(a9);
  a11=(a11*a18);
  a12=(a12+a11);
  a11=(a12-a7);
  a11=casadi_sq(a11);
  a14=(a14+a11);
  a14=sqrt(a14);
  a15=(a15-a14);
  a15=(-a15);
  a15=casadi_fmax(a0,a15);
  a14=casadi_sq(a10);
  a11=arg[0]? arg[0][7] : 0;
  a9=(a9+a11);
  a11=cos(a9);
  a11=(a11*a18);
  a11=(a11-a4);
  a11=casadi_sq(a11);
  a16=sin(a9);
  a16=(a16*a18);
  a16=(a16-a4);
  a16=casadi_sq(a16);
  a11=(a11+a16);
  a14=(a14*a11);
  a16=(a8-a1);
  a16=casadi_sq(a16);
  a17=(a12-a6);
  a17=casadi_sq(a17);
  a16=(a16+a17);
  a16=(a16*a11);
  a11=cos(a9);
  a11=(a11*a18);
  a11=(a11-a4);
  a17=(a8-a1);
  a11=(a11*a17);
  a17=sin(a9);
  a17=(a17*a18);
  a17=(a17-a4);
  a18=(a12-a6);
  a17=(a17*a18);
  a11=(a11+a17);
  a11=casadi_sq(a11);
  a16=(a16-a11);
  a14=(a14-a16);
  a14=casadi_fmax(a0,a14);
  a15=(a15*a14);
  a5=(a5+a15);
  a2=(a2+a13);
  a13=cos(a2);
  a13=(a13*a4);
  a1=(a1+a13);
  a13=(a1-a3);
  a13=casadi_sq(a13);
  a2=sin(a2);
  a2=(a2*a4);
  a6=(a6+a2);
  a2=(a6-a7);
  a2=casadi_sq(a2);
  a13=(a13+a2);
  a13=sqrt(a13);
  a2=cos(a9);
  a15=arg[0]? arg[0][8] : 0;
  a2=(a2*a15);
  a8=(a8+a2);
  a3=(a8-a3);
  a3=casadi_sq(a3);
  a2=sin(a9);
  a2=(a2*a15);
  a12=(a12+a2);
  a7=(a12-a7);
  a7=casadi_sq(a7);
  a3=(a3+a7);
  a3=sqrt(a3);
  a13=(a13-a3);
  a13=(-a13);
  a13=casadi_fmax(a0,a13);
  a10=casadi_sq(a10);
  a3=arg[0]? arg[0][9] : 0;
  a9=(a9+a3);
  a3=cos(a9);
  a3=(a3*a15);
  a3=(a3-a4);
  a3=casadi_sq(a3);
  a7=sin(a9);
  a7=(a7*a15);
  a7=(a7-a4);
  a7=casadi_sq(a7);
  a3=(a3+a7);
  a10=(a10*a3);
  a7=(a8-a1);
  a7=casadi_sq(a7);
  a2=(a12-a6);
  a2=casadi_sq(a2);
  a7=(a7+a2);
  a7=(a7*a3);
  a3=cos(a9);
  a3=(a3*a15);
  a3=(a3-a4);
  a8=(a8-a1);
  a3=(a3*a8);
  a9=sin(a9);
  a9=(a9*a15);
  a9=(a9-a4);
  a12=(a12-a6);
  a9=(a9*a12);
  a3=(a3+a9);
  a3=casadi_sq(a3);
  a7=(a7-a3);
  a10=(a10-a7);
  a0=casadi_fmax(a0,a10);
  a13=(a13*a0);
  a5=(a5+a13);
  if (res[0]!=0) res[0][0]=a5;
  return 0;
}

CASADI_SYMBOL_EXPORT int mapping_f1_InkASXGEZkQWNKKgVChq(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int mapping_f1_InkASXGEZkQWNKKgVChq_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int mapping_f1_InkASXGEZkQWNKKgVChq_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void mapping_f1_InkASXGEZkQWNKKgVChq_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int mapping_f1_InkASXGEZkQWNKKgVChq_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void mapping_f1_InkASXGEZkQWNKKgVChq_release(int mem) {
}

CASADI_SYMBOL_EXPORT void mapping_f1_InkASXGEZkQWNKKgVChq_incref(void) {
}

CASADI_SYMBOL_EXPORT void mapping_f1_InkASXGEZkQWNKKgVChq_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int mapping_f1_InkASXGEZkQWNKKgVChq_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int mapping_f1_InkASXGEZkQWNKKgVChq_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real mapping_f1_InkASXGEZkQWNKKgVChq_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* mapping_f1_InkASXGEZkQWNKKgVChq_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* mapping_f1_InkASXGEZkQWNKKgVChq_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* mapping_f1_InkASXGEZkQWNKKgVChq_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* mapping_f1_InkASXGEZkQWNKKgVChq_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int mapping_f1_InkASXGEZkQWNKKgVChq_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
