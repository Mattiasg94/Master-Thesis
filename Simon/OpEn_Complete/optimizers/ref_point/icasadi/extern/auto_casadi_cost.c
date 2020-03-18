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
  #define CASADI_PREFIX(ID) phi_kMghsVZrMDUmhSGiBatr_ ## ID
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
#define casadi_s3 CASADI_PREFIX(s3)
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

static const casadi_int casadi_s0[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* phi_kMghsVZrMDUmhSGiBatr:(i0[20],i1[2],i2[14])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a7, a8, a9;
  a0=10.;
  a1=arg[2]? arg[2][0] : 0;
  a2=arg[2]? arg[2][5] : 0;
  a3=(a1-a2);
  a3=casadi_sq(a3);
  a3=(a0*a3);
  a4=arg[2]? arg[2][1] : 0;
  a5=arg[2]? arg[2][6] : 0;
  a6=(a4-a5);
  a6=casadi_sq(a6);
  a6=(a0*a6);
  a3=(a3+a6);
  a6=arg[2]? arg[2][2] : 0;
  a7=cos(a6);
  a8=arg[0]? arg[0][0] : 0;
  a7=(a7*a8);
  a1=(a1+a7);
  a7=(a1-a2);
  a7=casadi_sq(a7);
  a7=(a0*a7);
  a9=sin(a6);
  a9=(a9*a8);
  a4=(a4+a9);
  a9=(a4-a5);
  a9=casadi_sq(a9);
  a9=(a0*a9);
  a7=(a7+a9);
  a3=(a3+a7);
  a7=arg[0]? arg[0][1] : 0;
  a6=(a6+a7);
  a9=cos(a6);
  a10=arg[0]? arg[0][2] : 0;
  a9=(a9*a10);
  a9=(a1+a9);
  a11=(a9-a2);
  a11=casadi_sq(a11);
  a11=(a0*a11);
  a12=sin(a6);
  a12=(a12*a10);
  a12=(a4+a12);
  a13=(a12-a5);
  a13=casadi_sq(a13);
  a13=(a0*a13);
  a11=(a11+a13);
  a3=(a3+a11);
  a11=arg[0]? arg[0][3] : 0;
  a13=(a6+a11);
  a14=cos(a13);
  a15=arg[0]? arg[0][4] : 0;
  a14=(a14*a15);
  a14=(a9+a14);
  a16=(a14-a2);
  a16=casadi_sq(a16);
  a16=(a0*a16);
  a17=sin(a13);
  a17=(a17*a15);
  a17=(a12+a17);
  a18=(a17-a5);
  a18=casadi_sq(a18);
  a18=(a0*a18);
  a16=(a16+a18);
  a3=(a3+a16);
  a16=arg[0]? arg[0][5] : 0;
  a18=(a13+a16);
  a19=cos(a18);
  a20=arg[0]? arg[0][6] : 0;
  a19=(a19*a20);
  a19=(a14+a19);
  a21=(a19-a2);
  a21=casadi_sq(a21);
  a21=(a0*a21);
  a22=sin(a18);
  a22=(a22*a20);
  a22=(a17+a22);
  a23=(a22-a5);
  a23=casadi_sq(a23);
  a23=(a0*a23);
  a21=(a21+a23);
  a3=(a3+a21);
  a21=arg[0]? arg[0][7] : 0;
  a23=(a18+a21);
  a24=cos(a23);
  a25=arg[0]? arg[0][8] : 0;
  a24=(a24*a25);
  a24=(a19+a24);
  a26=(a24-a2);
  a26=casadi_sq(a26);
  a26=(a0*a26);
  a27=sin(a23);
  a27=(a27*a25);
  a27=(a22+a27);
  a28=(a27-a5);
  a28=casadi_sq(a28);
  a28=(a0*a28);
  a26=(a26+a28);
  a3=(a3+a26);
  a26=arg[0]? arg[0][9] : 0;
  a28=(a23+a26);
  a29=cos(a28);
  a30=arg[0]? arg[0][10] : 0;
  a29=(a29*a30);
  a29=(a24+a29);
  a31=(a29-a2);
  a31=casadi_sq(a31);
  a31=(a0*a31);
  a32=sin(a28);
  a32=(a32*a30);
  a32=(a27+a32);
  a33=(a32-a5);
  a33=casadi_sq(a33);
  a33=(a0*a33);
  a31=(a31+a33);
  a3=(a3+a31);
  a31=arg[0]? arg[0][11] : 0;
  a33=(a28+a31);
  a34=cos(a33);
  a35=arg[0]? arg[0][12] : 0;
  a34=(a34*a35);
  a34=(a29+a34);
  a36=(a34-a2);
  a36=casadi_sq(a36);
  a36=(a0*a36);
  a37=sin(a33);
  a37=(a37*a35);
  a37=(a32+a37);
  a38=(a37-a5);
  a38=casadi_sq(a38);
  a38=(a0*a38);
  a36=(a36+a38);
  a3=(a3+a36);
  a36=arg[0]? arg[0][13] : 0;
  a38=(a33+a36);
  a39=cos(a38);
  a40=arg[0]? arg[0][14] : 0;
  a39=(a39*a40);
  a39=(a34+a39);
  a41=(a39-a2);
  a41=casadi_sq(a41);
  a41=(a0*a41);
  a42=sin(a38);
  a42=(a42*a40);
  a42=(a37+a42);
  a43=(a42-a5);
  a43=casadi_sq(a43);
  a43=(a0*a43);
  a41=(a41+a43);
  a3=(a3+a41);
  a41=arg[0]? arg[0][15] : 0;
  a43=(a38+a41);
  a44=cos(a43);
  a45=arg[0]? arg[0][16] : 0;
  a44=(a44*a45);
  a44=(a39+a44);
  a46=(a44-a2);
  a46=casadi_sq(a46);
  a46=(a0*a46);
  a47=sin(a43);
  a47=(a47*a45);
  a47=(a42+a47);
  a48=(a47-a5);
  a48=casadi_sq(a48);
  a48=(a0*a48);
  a46=(a46+a48);
  a3=(a3+a46);
  a46=arg[0]? arg[0][17] : 0;
  a48=(a43+a46);
  a49=cos(a48);
  a50=arg[0]? arg[0][18] : 0;
  a49=(a49*a50);
  a49=(a44+a49);
  a51=(a49-a2);
  a51=casadi_sq(a51);
  a51=(a0*a51);
  a52=sin(a48);
  a52=(a52*a50);
  a52=(a47+a52);
  a53=(a52-a5);
  a53=casadi_sq(a53);
  a53=(a0*a53);
  a51=(a51+a53);
  a53=arg[0]? arg[0][19] : 0;
  a54=(a48+a53);
  a55=arg[2]? arg[2][7] : 0;
  a55=(a54-a55);
  a55=casadi_sq(a55);
  a0=(a0*a55);
  a51=(a51+a0);
  a3=(a3+a51);
  a51=arg[1]? arg[1][0] : 0;
  a0=0.;
  a55=arg[2]? arg[2][8] : 0;
  a56=arg[2]? arg[2][10] : 0;
  a57=cos(a56);
  a58=arg[2]? arg[2][11] : 0;
  a57=(a57*a58);
  a55=(a55+a57);
  a57=(a55-a2);
  a57=casadi_sq(a57);
  a59=arg[2]? arg[2][9] : 0;
  a60=sin(a56);
  a60=(a60*a58);
  a59=(a59+a60);
  a60=(a59-a5);
  a60=casadi_sq(a60);
  a57=(a57+a60);
  a57=sqrt(a57);
  a60=(a1-a2);
  a60=casadi_sq(a60);
  a61=(a4-a5);
  a61=casadi_sq(a61);
  a60=(a60+a61);
  a60=sqrt(a60);
  a57=(a57-a60);
  a57=(-a57);
  a57=casadi_fmax(a0,a57);
  a60=arg[2]? arg[2][13] : 0;
  a61=casadi_sq(a60);
  a62=cos(a6);
  a62=(a62*a8);
  a62=(a62-a58);
  a62=casadi_sq(a62);
  a63=sin(a6);
  a63=(a63*a8);
  a63=(a63-a58);
  a63=casadi_sq(a63);
  a62=(a62+a63);
  a61=(a61*a62);
  a63=(a1-a55);
  a63=casadi_sq(a63);
  a64=(a4-a59);
  a64=casadi_sq(a64);
  a63=(a63+a64);
  a63=(a63*a62);
  a62=cos(a6);
  a62=(a62*a8);
  a62=(a62-a58);
  a1=(a1-a55);
  a62=(a62*a1);
  a6=sin(a6);
  a6=(a6*a8);
  a6=(a6-a58);
  a4=(a4-a59);
  a6=(a6*a4);
  a62=(a62+a6);
  a62=casadi_sq(a62);
  a63=(a63-a62);
  a61=(a61-a63);
  a61=casadi_fmax(a0,a61);
  a57=(a57*a61);
  a61=arg[2]? arg[2][12] : 0;
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a9-a2);
  a62=casadi_sq(a62);
  a6=(a12-a5);
  a6=casadi_sq(a6);
  a62=(a62+a6);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a6=cos(a13);
  a6=(a6*a10);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=sin(a13);
  a4=(a4*a10);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=(a6+a4);
  a62=(a62*a6);
  a4=(a9-a55);
  a4=casadi_sq(a4);
  a1=(a12-a59);
  a1=casadi_sq(a1);
  a4=(a4+a1);
  a4=(a4*a6);
  a6=cos(a13);
  a6=(a6*a10);
  a6=(a6-a58);
  a9=(a9-a55);
  a6=(a6*a9);
  a13=sin(a13);
  a13=(a13*a10);
  a13=(a13-a58);
  a12=(a12-a59);
  a13=(a13*a12);
  a6=(a6+a13);
  a6=casadi_sq(a6);
  a4=(a4-a6);
  a62=(a62-a4);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a14-a2);
  a62=casadi_sq(a62);
  a4=(a17-a5);
  a4=casadi_sq(a4);
  a62=(a62+a4);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a4=cos(a18);
  a4=(a4*a15);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=sin(a18);
  a6=(a6*a15);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=(a4+a6);
  a62=(a62*a4);
  a6=(a14-a55);
  a6=casadi_sq(a6);
  a13=(a17-a59);
  a13=casadi_sq(a13);
  a6=(a6+a13);
  a6=(a6*a4);
  a4=cos(a18);
  a4=(a4*a15);
  a4=(a4-a58);
  a14=(a14-a55);
  a4=(a4*a14);
  a18=sin(a18);
  a18=(a18*a15);
  a18=(a18-a58);
  a17=(a17-a59);
  a18=(a18*a17);
  a4=(a4+a18);
  a4=casadi_sq(a4);
  a6=(a6-a4);
  a62=(a62-a6);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a19-a2);
  a62=casadi_sq(a62);
  a6=(a22-a5);
  a6=casadi_sq(a6);
  a62=(a62+a6);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a6=cos(a23);
  a6=(a6*a20);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=sin(a23);
  a4=(a4*a20);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=(a6+a4);
  a62=(a62*a6);
  a4=(a19-a55);
  a4=casadi_sq(a4);
  a18=(a22-a59);
  a18=casadi_sq(a18);
  a4=(a4+a18);
  a4=(a4*a6);
  a6=cos(a23);
  a6=(a6*a20);
  a6=(a6-a58);
  a19=(a19-a55);
  a6=(a6*a19);
  a23=sin(a23);
  a23=(a23*a20);
  a23=(a23-a58);
  a22=(a22-a59);
  a23=(a23*a22);
  a6=(a6+a23);
  a6=casadi_sq(a6);
  a4=(a4-a6);
  a62=(a62-a4);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a24-a2);
  a62=casadi_sq(a62);
  a4=(a27-a5);
  a4=casadi_sq(a4);
  a62=(a62+a4);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a4=cos(a28);
  a4=(a4*a25);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=sin(a28);
  a6=(a6*a25);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=(a4+a6);
  a62=(a62*a4);
  a6=(a24-a55);
  a6=casadi_sq(a6);
  a23=(a27-a59);
  a23=casadi_sq(a23);
  a6=(a6+a23);
  a6=(a6*a4);
  a4=cos(a28);
  a4=(a4*a25);
  a4=(a4-a58);
  a24=(a24-a55);
  a4=(a4*a24);
  a28=sin(a28);
  a28=(a28*a25);
  a28=(a28-a58);
  a27=(a27-a59);
  a28=(a28*a27);
  a4=(a4+a28);
  a4=casadi_sq(a4);
  a6=(a6-a4);
  a62=(a62-a6);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a29-a2);
  a62=casadi_sq(a62);
  a6=(a32-a5);
  a6=casadi_sq(a6);
  a62=(a62+a6);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a6=cos(a33);
  a6=(a6*a30);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=sin(a33);
  a4=(a4*a30);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=(a6+a4);
  a62=(a62*a6);
  a4=(a29-a55);
  a4=casadi_sq(a4);
  a28=(a32-a59);
  a28=casadi_sq(a28);
  a4=(a4+a28);
  a4=(a4*a6);
  a6=cos(a33);
  a6=(a6*a30);
  a6=(a6-a58);
  a29=(a29-a55);
  a6=(a6*a29);
  a33=sin(a33);
  a33=(a33*a30);
  a33=(a33-a58);
  a32=(a32-a59);
  a33=(a33*a32);
  a6=(a6+a33);
  a6=casadi_sq(a6);
  a4=(a4-a6);
  a62=(a62-a4);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a34-a2);
  a62=casadi_sq(a62);
  a4=(a37-a5);
  a4=casadi_sq(a4);
  a62=(a62+a4);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a4=cos(a38);
  a4=(a4*a35);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=sin(a38);
  a6=(a6*a35);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=(a4+a6);
  a62=(a62*a4);
  a6=(a34-a55);
  a6=casadi_sq(a6);
  a33=(a37-a59);
  a33=casadi_sq(a33);
  a6=(a6+a33);
  a6=(a6*a4);
  a4=cos(a38);
  a4=(a4*a35);
  a4=(a4-a58);
  a34=(a34-a55);
  a4=(a4*a34);
  a38=sin(a38);
  a38=(a38*a35);
  a38=(a38-a58);
  a37=(a37-a59);
  a38=(a38*a37);
  a4=(a4+a38);
  a4=casadi_sq(a4);
  a6=(a6-a4);
  a62=(a62-a6);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a39-a2);
  a62=casadi_sq(a62);
  a6=(a42-a5);
  a6=casadi_sq(a6);
  a62=(a62+a6);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a6=cos(a43);
  a6=(a6*a40);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=sin(a43);
  a4=(a4*a40);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=(a6+a4);
  a62=(a62*a6);
  a4=(a39-a55);
  a4=casadi_sq(a4);
  a38=(a42-a59);
  a38=casadi_sq(a38);
  a4=(a4+a38);
  a4=(a4*a6);
  a6=cos(a43);
  a6=(a6*a40);
  a6=(a6-a58);
  a39=(a39-a55);
  a6=(a6*a39);
  a43=sin(a43);
  a43=(a43*a40);
  a43=(a43-a58);
  a42=(a42-a59);
  a43=(a43*a42);
  a6=(a6+a43);
  a6=casadi_sq(a6);
  a4=(a4-a6);
  a62=(a62-a4);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a63=cos(a56);
  a63=(a63*a58);
  a55=(a55+a63);
  a63=(a55-a2);
  a63=casadi_sq(a63);
  a62=sin(a56);
  a62=(a62*a58);
  a59=(a59+a62);
  a62=(a59-a5);
  a62=casadi_sq(a62);
  a63=(a63+a62);
  a63=sqrt(a63);
  a62=(a44-a2);
  a62=casadi_sq(a62);
  a4=(a47-a5);
  a4=casadi_sq(a4);
  a62=(a62+a4);
  a62=sqrt(a62);
  a63=(a63-a62);
  a63=(-a63);
  a63=casadi_fmax(a0,a63);
  a62=casadi_sq(a60);
  a4=cos(a48);
  a4=(a4*a45);
  a4=(a4-a58);
  a4=casadi_sq(a4);
  a6=sin(a48);
  a6=(a6*a45);
  a6=(a6-a58);
  a6=casadi_sq(a6);
  a4=(a4+a6);
  a62=(a62*a4);
  a6=(a44-a55);
  a6=casadi_sq(a6);
  a43=(a47-a59);
  a43=casadi_sq(a43);
  a6=(a6+a43);
  a6=(a6*a4);
  a4=cos(a48);
  a4=(a4*a45);
  a4=(a4-a58);
  a44=(a44-a55);
  a4=(a4*a44);
  a48=sin(a48);
  a48=(a48*a45);
  a48=(a48-a58);
  a47=(a47-a59);
  a48=(a48*a47);
  a4=(a4+a48);
  a4=casadi_sq(a4);
  a6=(a6-a4);
  a62=(a62-a6);
  a62=casadi_fmax(a0,a62);
  a63=(a63*a62);
  a57=(a57+a63);
  a56=(a56+a61);
  a61=cos(a56);
  a61=(a61*a58);
  a55=(a55+a61);
  a61=(a55-a2);
  a61=casadi_sq(a61);
  a56=sin(a56);
  a56=(a56*a58);
  a59=(a59+a56);
  a56=(a59-a5);
  a56=casadi_sq(a56);
  a61=(a61+a56);
  a61=sqrt(a61);
  a2=(a49-a2);
  a2=casadi_sq(a2);
  a5=(a52-a5);
  a5=casadi_sq(a5);
  a2=(a2+a5);
  a2=sqrt(a2);
  a61=(a61-a2);
  a61=(-a61);
  a61=casadi_fmax(a0,a61);
  a60=casadi_sq(a60);
  a2=cos(a54);
  a2=(a2*a50);
  a2=(a2-a58);
  a2=casadi_sq(a2);
  a5=sin(a54);
  a5=(a5*a50);
  a5=(a5-a58);
  a5=casadi_sq(a5);
  a2=(a2+a5);
  a60=(a60*a2);
  a5=(a49-a55);
  a5=casadi_sq(a5);
  a56=(a52-a59);
  a56=casadi_sq(a56);
  a5=(a5+a56);
  a5=(a5*a2);
  a2=cos(a54);
  a2=(a2*a50);
  a2=(a2-a58);
  a49=(a49-a55);
  a2=(a2*a49);
  a54=sin(a54);
  a54=(a54*a50);
  a54=(a54-a58);
  a52=(a52-a59);
  a54=(a54*a52);
  a2=(a2+a54);
  a2=casadi_sq(a2);
  a5=(a5-a2);
  a60=(a60-a5);
  a60=casadi_fmax(a0,a60);
  a61=(a61*a60);
  a57=(a57+a61);
  a61=arg[1]? arg[1][1] : 0;
  a61=(a61/a51);
  a57=(a57+a61);
  a57=casadi_sq(a57);
  a57=(a51*a57);
  a61=2.;
  a57=(a57/a61);
  a3=(a3+a57);
  a57=arg[2]? arg[2][3] : 0;
  a57=(a8-a57);
  a60=1.0000000000000001e-001;
  a57=(a57-a60);
  a57=casadi_fmax(a0,a57);
  a8=(a10-a8);
  a8=(a8-a60);
  a8=casadi_fmax(a0,a8);
  a57=(a57+a8);
  a10=(a15-a10);
  a10=(a10-a60);
  a10=casadi_fmax(a0,a10);
  a57=(a57+a10);
  a15=(a20-a15);
  a15=(a15-a60);
  a15=casadi_fmax(a0,a15);
  a57=(a57+a15);
  a20=(a25-a20);
  a20=(a20-a60);
  a20=casadi_fmax(a0,a20);
  a57=(a57+a20);
  a25=(a30-a25);
  a25=(a25-a60);
  a25=casadi_fmax(a0,a25);
  a57=(a57+a25);
  a30=(a35-a30);
  a30=(a30-a60);
  a30=casadi_fmax(a0,a30);
  a57=(a57+a30);
  a35=(a40-a35);
  a35=(a35-a60);
  a35=casadi_fmax(a0,a35);
  a57=(a57+a35);
  a40=(a45-a40);
  a40=(a40-a60);
  a40=casadi_fmax(a0,a40);
  a57=(a57+a40);
  a50=(a50-a45);
  a50=(a50-a60);
  a50=casadi_fmax(a0,a50);
  a57=(a57+a50);
  a57=casadi_sq(a57);
  a50=arg[2]? arg[2][4] : 0;
  a60=(a7-a50);
  a45=5.0000000000000000e-001;
  a60=(a60-a45);
  a60=casadi_fmax(a0,a60);
  a40=(a11-a7);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a16-a11);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a21-a16);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a26-a21);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a31-a26);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a36-a31);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a41-a36);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a46-a41);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a40=(a53-a46);
  a40=(a40-a45);
  a40=casadi_fmax(a0,a40);
  a60=(a60+a40);
  a60=casadi_sq(a60);
  a57=(a57+a60);
  a50=(a50-a7);
  a50=(a50-a45);
  a50=casadi_fmax(a0,a50);
  a7=(a7-a11);
  a7=(a7-a45);
  a7=casadi_fmax(a0,a7);
  a50=(a50+a7);
  a11=(a11-a16);
  a11=(a11-a45);
  a11=casadi_fmax(a0,a11);
  a50=(a50+a11);
  a16=(a16-a21);
  a16=(a16-a45);
  a16=casadi_fmax(a0,a16);
  a50=(a50+a16);
  a21=(a21-a26);
  a21=(a21-a45);
  a21=casadi_fmax(a0,a21);
  a50=(a50+a21);
  a26=(a26-a31);
  a26=(a26-a45);
  a26=casadi_fmax(a0,a26);
  a50=(a50+a26);
  a31=(a31-a36);
  a31=(a31-a45);
  a31=casadi_fmax(a0,a31);
  a50=(a50+a31);
  a36=(a36-a41);
  a36=(a36-a45);
  a36=casadi_fmax(a0,a36);
  a50=(a50+a36);
  a41=(a41-a46);
  a41=(a41-a45);
  a41=casadi_fmax(a0,a41);
  a50=(a50+a41);
  a46=(a46-a53);
  a46=(a46-a45);
  a0=casadi_fmax(a0,a46);
  a50=(a50+a0);
  a50=casadi_sq(a50);
  a57=(a57+a50);
  a51=(a51*a57);
  a51=(a51/a61);
  a3=(a3+a51);
  if (res[0]!=0) res[0][0]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int phi_kMghsVZrMDUmhSGiBatr(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int phi_kMghsVZrMDUmhSGiBatr_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int phi_kMghsVZrMDUmhSGiBatr_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void phi_kMghsVZrMDUmhSGiBatr_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int phi_kMghsVZrMDUmhSGiBatr_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void phi_kMghsVZrMDUmhSGiBatr_release(int mem) {
}

CASADI_SYMBOL_EXPORT void phi_kMghsVZrMDUmhSGiBatr_incref(void) {
}

CASADI_SYMBOL_EXPORT void phi_kMghsVZrMDUmhSGiBatr_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int phi_kMghsVZrMDUmhSGiBatr_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int phi_kMghsVZrMDUmhSGiBatr_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real phi_kMghsVZrMDUmhSGiBatr_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* phi_kMghsVZrMDUmhSGiBatr_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* phi_kMghsVZrMDUmhSGiBatr_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* phi_kMghsVZrMDUmhSGiBatr_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* phi_kMghsVZrMDUmhSGiBatr_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int phi_kMghsVZrMDUmhSGiBatr_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif