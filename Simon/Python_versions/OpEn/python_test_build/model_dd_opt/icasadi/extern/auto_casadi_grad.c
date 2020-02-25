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
  #define CASADI_PREFIX(ID) grad_phi_mVnUdMpFCOEqLCsYCCQs_ ## ID
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

static const casadi_int casadi_s0[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[7] = {3, 1, 0, 3, 0, 1, 2};

/* grad_phi_mVnUdMpFCOEqLCsYCCQs:(i0[20],i1,i2[3])->(o0[20]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a9;
  a0=1.0000000000000001e-001;
  a1=arg[2]? arg[2][2] : 0;
  a2=sin(a1);
  a2=(a0*a2);
  a3=100.;
  a4=arg[2]? arg[2][1] : 0;
  a5=arg[0]? arg[0][0] : 0;
  a6=(a2*a5);
  a4=(a4+a6);
  a6=arg[0]? arg[0][1] : 0;
  a7=(a0*a6);
  a7=(a1+a7);
  a8=sin(a7);
  a8=(a0*a8);
  a9=arg[0]? arg[0][2] : 0;
  a10=(a8*a9);
  a10=(a4+a10);
  a11=arg[0]? arg[0][3] : 0;
  a12=(a0*a11);
  a12=(a7+a12);
  a13=sin(a12);
  a13=(a0*a13);
  a14=arg[0]? arg[0][4] : 0;
  a15=(a13*a14);
  a15=(a10+a15);
  a16=arg[0]? arg[0][5] : 0;
  a17=(a0*a16);
  a17=(a12+a17);
  a18=sin(a17);
  a18=(a0*a18);
  a19=arg[0]? arg[0][6] : 0;
  a20=(a18*a19);
  a20=(a15+a20);
  a21=arg[0]? arg[0][7] : 0;
  a22=(a0*a21);
  a22=(a17+a22);
  a23=sin(a22);
  a23=(a0*a23);
  a24=arg[0]? arg[0][8] : 0;
  a25=(a23*a24);
  a25=(a20+a25);
  a26=arg[0]? arg[0][9] : 0;
  a27=(a0*a26);
  a27=(a22+a27);
  a28=sin(a27);
  a28=(a0*a28);
  a29=arg[0]? arg[0][10] : 0;
  a30=(a28*a29);
  a30=(a25+a30);
  a31=arg[0]? arg[0][11] : 0;
  a32=(a0*a31);
  a32=(a27+a32);
  a33=sin(a32);
  a33=(a0*a33);
  a34=arg[0]? arg[0][12] : 0;
  a35=(a33*a34);
  a35=(a30+a35);
  a36=arg[0]? arg[0][13] : 0;
  a37=(a0*a36);
  a37=(a32+a37);
  a38=sin(a37);
  a38=(a0*a38);
  a39=arg[0]? arg[0][14] : 0;
  a40=(a38*a39);
  a40=(a35+a40);
  a41=arg[0]? arg[0][15] : 0;
  a42=(a0*a41);
  a42=(a37+a42);
  a43=sin(a42);
  a43=(a0*a43);
  a44=arg[0]? arg[0][16] : 0;
  a45=(a43*a44);
  a45=(a40+a45);
  a46=arg[0]? arg[0][17] : 0;
  a47=(a0*a46);
  a47=(a42+a47);
  a48=sin(a47);
  a48=(a0*a48);
  a49=arg[0]? arg[0][18] : 0;
  a50=(a48*a49);
  a50=(a45+a50);
  a51=2.;
  a52=(a50-a51);
  a52=(a52+a52);
  a52=(a3*a52);
  a53=(a50+a50);
  a54=5.0000000000000000e-001;
  a55=arg[2]? arg[2][0] : 0;
  a1=cos(a1);
  a1=(a0*a1);
  a56=(a1*a5);
  a55=(a55+a56);
  a56=cos(a7);
  a56=(a0*a56);
  a57=(a56*a9);
  a57=(a55+a57);
  a58=cos(a12);
  a58=(a0*a58);
  a59=(a58*a14);
  a59=(a57+a59);
  a60=cos(a17);
  a60=(a0*a60);
  a61=(a60*a19);
  a61=(a59+a61);
  a62=cos(a22);
  a62=(a0*a62);
  a63=(a62*a24);
  a63=(a61+a63);
  a64=cos(a27);
  a64=(a0*a64);
  a65=(a64*a29);
  a65=(a63+a65);
  a66=cos(a32);
  a66=(a0*a66);
  a67=(a66*a34);
  a67=(a65+a67);
  a68=cos(a37);
  a68=(a0*a68);
  a69=(a68*a39);
  a69=(a67+a69);
  a70=cos(a42);
  a70=(a0*a70);
  a71=(a70*a44);
  a71=(a69+a71);
  a72=cos(a47);
  a72=(a0*a72);
  a73=(a72*a49);
  a73=(a71+a73);
  a74=casadi_sq(a73);
  a74=(a54-a74);
  a50=casadi_sq(a50);
  a74=(a74-a50);
  a50=0.;
  a75=(a74<=a50);
  a75=(!a75);
  a76=casadi_sq(a55);
  a76=(a54-a76);
  a77=casadi_sq(a4);
  a76=(a76-a77);
  a77=casadi_fmax(a50,a76);
  a78=casadi_sq(a57);
  a78=(a54-a78);
  a79=casadi_sq(a10);
  a78=(a78-a79);
  a79=casadi_fmax(a50,a78);
  a77=(a77+a79);
  a79=casadi_sq(a59);
  a79=(a54-a79);
  a80=casadi_sq(a15);
  a79=(a79-a80);
  a80=casadi_fmax(a50,a79);
  a77=(a77+a80);
  a80=casadi_sq(a61);
  a80=(a54-a80);
  a81=casadi_sq(a20);
  a80=(a80-a81);
  a81=casadi_fmax(a50,a80);
  a77=(a77+a81);
  a81=casadi_sq(a63);
  a81=(a54-a81);
  a82=casadi_sq(a25);
  a81=(a81-a82);
  a82=casadi_fmax(a50,a81);
  a77=(a77+a82);
  a82=casadi_sq(a65);
  a82=(a54-a82);
  a83=casadi_sq(a30);
  a82=(a82-a83);
  a83=casadi_fmax(a50,a82);
  a77=(a77+a83);
  a83=casadi_sq(a67);
  a83=(a54-a83);
  a84=casadi_sq(a35);
  a83=(a83-a84);
  a84=casadi_fmax(a50,a83);
  a77=(a77+a84);
  a84=casadi_sq(a69);
  a84=(a54-a84);
  a85=casadi_sq(a40);
  a84=(a84-a85);
  a85=casadi_fmax(a50,a84);
  a77=(a77+a85);
  a85=casadi_sq(a71);
  a85=(a54-a85);
  a86=casadi_sq(a45);
  a85=(a85-a86);
  a86=casadi_fmax(a50,a85);
  a77=(a77+a86);
  a74=casadi_fmax(a50,a74);
  a77=(a77+a74);
  a77=(a77+a77);
  a74=arg[1]? arg[1][0] : 0;
  a54=(a54*a74);
  a77=(a77*a54);
  a75=(a75*a77);
  a53=(a53*a75);
  a52=(a52-a53);
  a53=(a45+a45);
  a85=(a85<=a50);
  a85=(!a85);
  a85=(a85*a77);
  a53=(a53*a85);
  a53=(a52-a53);
  a54=10.;
  a45=(a45-a51);
  a45=(a45+a45);
  a45=(a54*a45);
  a53=(a53+a45);
  a45=(a40+a40);
  a84=(a84<=a50);
  a84=(!a84);
  a84=(a84*a77);
  a45=(a45*a84);
  a45=(a53-a45);
  a40=(a40-a51);
  a40=(a40+a40);
  a40=(a54*a40);
  a45=(a45+a40);
  a40=(a35+a35);
  a83=(a83<=a50);
  a83=(!a83);
  a83=(a83*a77);
  a40=(a40*a83);
  a40=(a45-a40);
  a35=(a35-a51);
  a35=(a35+a35);
  a35=(a54*a35);
  a40=(a40+a35);
  a35=(a30+a30);
  a82=(a82<=a50);
  a82=(!a82);
  a82=(a82*a77);
  a35=(a35*a82);
  a35=(a40-a35);
  a30=(a30-a51);
  a30=(a30+a30);
  a30=(a54*a30);
  a35=(a35+a30);
  a30=(a25+a25);
  a81=(a81<=a50);
  a81=(!a81);
  a81=(a81*a77);
  a30=(a30*a81);
  a30=(a35-a30);
  a25=(a25-a51);
  a25=(a25+a25);
  a25=(a54*a25);
  a30=(a30+a25);
  a25=(a20+a20);
  a80=(a80<=a50);
  a80=(!a80);
  a80=(a80*a77);
  a25=(a25*a80);
  a25=(a30-a25);
  a20=(a20-a51);
  a20=(a20+a20);
  a20=(a54*a20);
  a25=(a25+a20);
  a20=(a15+a15);
  a79=(a79<=a50);
  a79=(!a79);
  a79=(a79*a77);
  a20=(a20*a79);
  a20=(a25-a20);
  a15=(a15-a51);
  a15=(a15+a15);
  a15=(a54*a15);
  a20=(a20+a15);
  a15=(a10+a10);
  a78=(a78<=a50);
  a78=(!a78);
  a78=(a78*a77);
  a15=(a15*a78);
  a15=(a20-a15);
  a10=(a10-a51);
  a10=(a10+a10);
  a10=(a54*a10);
  a15=(a15+a10);
  a10=(a4+a4);
  a76=(a76<=a50);
  a76=(!a76);
  a76=(a76*a77);
  a10=(a10*a76);
  a10=(a15-a10);
  a4=(a4-a51);
  a4=(a4+a4);
  a4=(a54*a4);
  a10=(a10+a4);
  a2=(a2*a10);
  a10=(a73-a51);
  a10=(a10+a10);
  a3=(a3*a10);
  a73=(a73+a73);
  a73=(a73*a75);
  a3=(a3-a73);
  a73=(a71+a71);
  a73=(a73*a85);
  a73=(a3-a73);
  a71=(a71-a51);
  a71=(a71+a71);
  a71=(a54*a71);
  a73=(a73+a71);
  a71=(a69+a69);
  a71=(a71*a84);
  a71=(a73-a71);
  a69=(a69-a51);
  a69=(a69+a69);
  a69=(a54*a69);
  a71=(a71+a69);
  a69=(a67+a67);
  a69=(a69*a83);
  a69=(a71-a69);
  a67=(a67-a51);
  a67=(a67+a67);
  a67=(a54*a67);
  a69=(a69+a67);
  a67=(a65+a65);
  a67=(a67*a82);
  a67=(a69-a67);
  a65=(a65-a51);
  a65=(a65+a65);
  a65=(a54*a65);
  a67=(a67+a65);
  a65=(a63+a63);
  a65=(a65*a81);
  a65=(a67-a65);
  a63=(a63-a51);
  a63=(a63+a63);
  a63=(a54*a63);
  a65=(a65+a63);
  a63=(a61+a61);
  a63=(a63*a80);
  a63=(a65-a63);
  a61=(a61-a51);
  a61=(a61+a61);
  a61=(a54*a61);
  a63=(a63+a61);
  a61=(a59+a59);
  a61=(a61*a79);
  a61=(a63-a61);
  a59=(a59-a51);
  a59=(a59+a59);
  a59=(a54*a59);
  a61=(a61+a59);
  a59=(a57+a57);
  a59=(a59*a78);
  a59=(a61-a59);
  a57=(a57-a51);
  a57=(a57+a57);
  a57=(a54*a57);
  a59=(a59+a57);
  a57=(a55+a55);
  a57=(a57*a76);
  a57=(a59-a57);
  a55=(a55-a51);
  a55=(a55+a55);
  a54=(a54*a55);
  a57=(a57+a54);
  a1=(a1*a57);
  a2=(a2+a1);
  a5=(a5+a5);
  a2=(a2+a5);
  if (res[0]!=0) res[0][0]=a2;
  a2=arg[0]? arg[0][19] : 0;
  a5=(a0*a2);
  a5=(a47+a5);
  a5=(a5+a5);
  a1=cos(a47);
  a57=(a49*a52);
  a57=(a0*a57);
  a1=(a1*a57);
  a1=(a5+a1);
  a57=sin(a47);
  a54=(a49*a3);
  a54=(a0*a54);
  a57=(a57*a54);
  a1=(a1-a57);
  a47=(a47+a47);
  a1=(a1+a47);
  a47=cos(a42);
  a57=(a44*a53);
  a57=(a0*a57);
  a47=(a47*a57);
  a47=(a1+a47);
  a57=sin(a42);
  a54=(a44*a73);
  a54=(a0*a54);
  a57=(a57*a54);
  a47=(a47-a57);
  a42=(a42+a42);
  a47=(a47+a42);
  a42=cos(a37);
  a57=(a39*a45);
  a57=(a0*a57);
  a42=(a42*a57);
  a42=(a47+a42);
  a57=sin(a37);
  a54=(a39*a71);
  a54=(a0*a54);
  a57=(a57*a54);
  a42=(a42-a57);
  a37=(a37+a37);
  a42=(a42+a37);
  a37=cos(a32);
  a57=(a34*a40);
  a57=(a0*a57);
  a37=(a37*a57);
  a37=(a42+a37);
  a57=sin(a32);
  a54=(a34*a69);
  a54=(a0*a54);
  a57=(a57*a54);
  a37=(a37-a57);
  a32=(a32+a32);
  a37=(a37+a32);
  a32=cos(a27);
  a57=(a29*a35);
  a57=(a0*a57);
  a32=(a32*a57);
  a32=(a37+a32);
  a57=sin(a27);
  a54=(a29*a67);
  a54=(a0*a54);
  a57=(a57*a54);
  a32=(a32-a57);
  a27=(a27+a27);
  a32=(a32+a27);
  a27=cos(a22);
  a57=(a24*a30);
  a57=(a0*a57);
  a27=(a27*a57);
  a27=(a32+a27);
  a57=sin(a22);
  a54=(a24*a65);
  a54=(a0*a54);
  a57=(a57*a54);
  a27=(a27-a57);
  a22=(a22+a22);
  a27=(a27+a22);
  a22=cos(a17);
  a57=(a19*a25);
  a57=(a0*a57);
  a22=(a22*a57);
  a22=(a27+a22);
  a57=sin(a17);
  a54=(a19*a63);
  a54=(a0*a54);
  a57=(a57*a54);
  a22=(a22-a57);
  a17=(a17+a17);
  a22=(a22+a17);
  a17=cos(a12);
  a57=(a14*a20);
  a57=(a0*a57);
  a17=(a17*a57);
  a17=(a22+a17);
  a57=sin(a12);
  a54=(a14*a61);
  a54=(a0*a54);
  a57=(a57*a54);
  a17=(a17-a57);
  a12=(a12+a12);
  a17=(a17+a12);
  a12=cos(a7);
  a57=(a9*a15);
  a57=(a0*a57);
  a12=(a12*a57);
  a12=(a17+a12);
  a57=sin(a7);
  a54=(a9*a59);
  a54=(a0*a54);
  a57=(a57*a54);
  a12=(a12-a57);
  a7=(a7+a7);
  a12=(a12+a7);
  a12=(a0*a12);
  a6=(a6+a6);
  a12=(a12+a6);
  if (res[0]!=0) res[0][1]=a12;
  a8=(a8*a15);
  a56=(a56*a59);
  a8=(a8+a56);
  a9=(a9+a9);
  a8=(a8+a9);
  if (res[0]!=0) res[0][2]=a8;
  a17=(a0*a17);
  a11=(a11+a11);
  a17=(a17+a11);
  if (res[0]!=0) res[0][3]=a17;
  a13=(a13*a20);
  a58=(a58*a61);
  a13=(a13+a58);
  a14=(a14+a14);
  a13=(a13+a14);
  if (res[0]!=0) res[0][4]=a13;
  a22=(a0*a22);
  a16=(a16+a16);
  a22=(a22+a16);
  if (res[0]!=0) res[0][5]=a22;
  a18=(a18*a25);
  a60=(a60*a63);
  a18=(a18+a60);
  a19=(a19+a19);
  a18=(a18+a19);
  if (res[0]!=0) res[0][6]=a18;
  a27=(a0*a27);
  a21=(a21+a21);
  a27=(a27+a21);
  if (res[0]!=0) res[0][7]=a27;
  a23=(a23*a30);
  a62=(a62*a65);
  a23=(a23+a62);
  a24=(a24+a24);
  a23=(a23+a24);
  if (res[0]!=0) res[0][8]=a23;
  a32=(a0*a32);
  a26=(a26+a26);
  a32=(a32+a26);
  if (res[0]!=0) res[0][9]=a32;
  a28=(a28*a35);
  a64=(a64*a67);
  a28=(a28+a64);
  a29=(a29+a29);
  a28=(a28+a29);
  if (res[0]!=0) res[0][10]=a28;
  a37=(a0*a37);
  a31=(a31+a31);
  a37=(a37+a31);
  if (res[0]!=0) res[0][11]=a37;
  a33=(a33*a40);
  a66=(a66*a69);
  a33=(a33+a66);
  a34=(a34+a34);
  a33=(a33+a34);
  if (res[0]!=0) res[0][12]=a33;
  a42=(a0*a42);
  a36=(a36+a36);
  a42=(a42+a36);
  if (res[0]!=0) res[0][13]=a42;
  a38=(a38*a45);
  a68=(a68*a71);
  a38=(a38+a68);
  a39=(a39+a39);
  a38=(a38+a39);
  if (res[0]!=0) res[0][14]=a38;
  a47=(a0*a47);
  a41=(a41+a41);
  a47=(a47+a41);
  if (res[0]!=0) res[0][15]=a47;
  a43=(a43*a53);
  a70=(a70*a73);
  a43=(a43+a70);
  a44=(a44+a44);
  a43=(a43+a44);
  if (res[0]!=0) res[0][16]=a43;
  a1=(a0*a1);
  a46=(a46+a46);
  a1=(a1+a46);
  if (res[0]!=0) res[0][17]=a1;
  a48=(a48*a52);
  a72=(a72*a3);
  a48=(a48+a72);
  a49=(a49+a49);
  a48=(a48+a49);
  if (res[0]!=0) res[0][18]=a48;
  a0=(a0*a5);
  a2=(a2+a2);
  a0=(a0+a2);
  if (res[0]!=0) res[0][19]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_mVnUdMpFCOEqLCsYCCQs(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int grad_phi_mVnUdMpFCOEqLCsYCCQs_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_mVnUdMpFCOEqLCsYCCQs_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_mVnUdMpFCOEqLCsYCCQs_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int grad_phi_mVnUdMpFCOEqLCsYCCQs_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_mVnUdMpFCOEqLCsYCCQs_release(int mem) {
}

CASADI_SYMBOL_EXPORT void grad_phi_mVnUdMpFCOEqLCsYCCQs_incref(void) {
}

CASADI_SYMBOL_EXPORT void grad_phi_mVnUdMpFCOEqLCsYCCQs_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_mVnUdMpFCOEqLCsYCCQs_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_mVnUdMpFCOEqLCsYCCQs_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real grad_phi_mVnUdMpFCOEqLCsYCCQs_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_mVnUdMpFCOEqLCsYCCQs_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_mVnUdMpFCOEqLCsYCCQs_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_mVnUdMpFCOEqLCsYCCQs_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_mVnUdMpFCOEqLCsYCCQs_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int grad_phi_mVnUdMpFCOEqLCsYCCQs_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif