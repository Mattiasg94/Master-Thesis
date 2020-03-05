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
  #define CASADI_PREFIX(ID) grad_phi_InkASXGEZkQWNKKgVChq_ ## ID
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

casadi_real casadi_fmax(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>y ? x : y;
#else
  return fmax(x, y);
#endif
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

/* grad_phi_InkASXGEZkQWNKKgVChq:(i0[10],i1[2],i2[14])->(o0[10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a120, a121, a122, a123, a124, a125, a126, a127, a128, a129, a13, a130, a131, a132, a133, a134, a135, a136, a137, a138, a139, a14, a140, a141, a142, a143, a144, a145, a146, a147, a148, a149, a15, a150, a151, a152, a153, a154, a155, a156, a157, a158, a159, a16, a160, a161, a162, a163, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=arg[0]? arg[0][0] : 0;
  a1=arg[2]? arg[2][3] : 0;
  a1=(a0-a1);
  a2=1.0000000000000001e-001;
  a1=(a1-a2);
  a3=0.;
  a4=(a1<=a3);
  a4=(!a4);
  a1=casadi_fmax(a3,a1);
  a5=arg[0]? arg[0][2] : 0;
  a6=(a5-a0);
  a6=(a6-a2);
  a7=casadi_fmax(a3,a6);
  a1=(a1+a7);
  a7=arg[0]? arg[0][4] : 0;
  a8=(a7-a5);
  a8=(a8-a2);
  a9=casadi_fmax(a3,a8);
  a1=(a1+a9);
  a9=arg[0]? arg[0][6] : 0;
  a10=(a9-a7);
  a10=(a10-a2);
  a11=casadi_fmax(a3,a10);
  a1=(a1+a11);
  a11=arg[0]? arg[0][8] : 0;
  a12=(a11-a9);
  a12=(a12-a2);
  a2=casadi_fmax(a3,a12);
  a1=(a1+a2);
  a1=(a1+a1);
  a2=5.0000000000000000e-001;
  a13=arg[1]? arg[1][0] : 0;
  a14=(a2*a13);
  a1=(a1*a14);
  a4=(a4*a1);
  a6=(a6<=a3);
  a6=(!a6);
  a6=(a6*a1);
  a4=(a4-a6);
  a15=arg[2]? arg[2][2] : 0;
  a16=arg[0]? arg[0][1] : 0;
  a17=(a15+a16);
  a18=sin(a17);
  a19=arg[2]? arg[2][1] : 0;
  a20=sin(a15);
  a21=(a20*a0);
  a19=(a19+a21);
  a21=arg[2]? arg[2][9] : 0;
  a22=arg[2]? arg[2][10] : 0;
  a23=sin(a22);
  a24=arg[2]? arg[2][11] : 0;
  a23=(a23*a24);
  a21=(a21+a23);
  a23=(a19-a21);
  a25=cos(a17);
  a26=(a25*a0);
  a26=(a26-a24);
  a27=arg[2]? arg[2][0] : 0;
  a15=cos(a15);
  a28=(a15*a0);
  a27=(a27+a28);
  a28=arg[2]? arg[2][8] : 0;
  a29=cos(a22);
  a29=(a29*a24);
  a28=(a28+a29);
  a29=(a27-a28);
  a30=(a26*a29);
  a31=(a18*a0);
  a31=(a31-a24);
  a32=(a31*a23);
  a30=(a30+a32);
  a32=(a30+a30);
  a33=arg[2]? arg[2][13] : 0;
  a34=casadi_sq(a33);
  a35=cos(a17);
  a36=(a35*a0);
  a36=(a36-a24);
  a37=casadi_sq(a36);
  a38=sin(a17);
  a39=(a38*a0);
  a39=(a39-a24);
  a40=casadi_sq(a39);
  a37=(a37+a40);
  a40=(a34*a37);
  a41=(a27-a28);
  a42=casadi_sq(a41);
  a43=(a19-a21);
  a44=casadi_sq(a43);
  a42=(a42+a44);
  a44=(a42*a37);
  a30=casadi_sq(a30);
  a44=(a44-a30);
  a40=(a40-a44);
  a44=(a40<=a3);
  a44=(!a44);
  a30=arg[2]? arg[2][5] : 0;
  a45=(a28-a30);
  a45=casadi_sq(a45);
  a46=arg[2]? arg[2][6] : 0;
  a47=(a21-a46);
  a47=casadi_sq(a47);
  a45=(a45+a47);
  a45=sqrt(a45);
  a47=(a27-a30);
  a48=casadi_sq(a47);
  a49=(a19-a46);
  a50=casadi_sq(a49);
  a48=(a48+a50);
  a48=sqrt(a48);
  a45=(a45-a48);
  a45=(-a45);
  a50=casadi_fmax(a3,a45);
  a40=casadi_fmax(a3,a40);
  a51=(a50*a40);
  a52=arg[2]? arg[2][12] : 0;
  a22=(a22+a52);
  a53=cos(a22);
  a53=(a53*a24);
  a28=(a28+a53);
  a53=(a28-a30);
  a53=casadi_sq(a53);
  a54=sin(a22);
  a54=(a54*a24);
  a21=(a21+a54);
  a54=(a21-a46);
  a54=casadi_sq(a54);
  a53=(a53+a54);
  a53=sqrt(a53);
  a54=cos(a17);
  a55=(a54*a5);
  a55=(a27+a55);
  a56=(a55-a30);
  a57=casadi_sq(a56);
  a58=sin(a17);
  a59=(a58*a5);
  a59=(a19+a59);
  a60=(a59-a46);
  a61=casadi_sq(a60);
  a57=(a57+a61);
  a57=sqrt(a57);
  a53=(a53-a57);
  a53=(-a53);
  a61=casadi_fmax(a3,a53);
  a62=casadi_sq(a33);
  a63=arg[0]? arg[0][3] : 0;
  a64=(a17+a63);
  a65=cos(a64);
  a66=(a65*a5);
  a66=(a66-a24);
  a67=casadi_sq(a66);
  a68=sin(a64);
  a69=(a68*a5);
  a69=(a69-a24);
  a70=casadi_sq(a69);
  a67=(a67+a70);
  a70=(a62*a67);
  a71=(a55-a28);
  a72=casadi_sq(a71);
  a73=(a59-a21);
  a74=casadi_sq(a73);
  a72=(a72+a74);
  a74=(a72*a67);
  a75=cos(a64);
  a76=(a75*a5);
  a76=(a76-a24);
  a77=(a55-a28);
  a78=(a76*a77);
  a79=sin(a64);
  a80=(a79*a5);
  a80=(a80-a24);
  a81=(a59-a21);
  a82=(a80*a81);
  a78=(a78+a82);
  a82=casadi_sq(a78);
  a74=(a74-a82);
  a70=(a70-a74);
  a74=casadi_fmax(a3,a70);
  a82=(a61*a74);
  a51=(a51+a82);
  a22=(a22+a52);
  a82=cos(a22);
  a82=(a82*a24);
  a28=(a28+a82);
  a82=(a28-a30);
  a82=casadi_sq(a82);
  a83=sin(a22);
  a83=(a83*a24);
  a21=(a21+a83);
  a83=(a21-a46);
  a83=casadi_sq(a83);
  a82=(a82+a83);
  a82=sqrt(a82);
  a83=cos(a64);
  a84=(a83*a7);
  a84=(a55+a84);
  a85=(a84-a30);
  a86=casadi_sq(a85);
  a87=sin(a64);
  a88=(a87*a7);
  a88=(a59+a88);
  a89=(a88-a46);
  a90=casadi_sq(a89);
  a86=(a86+a90);
  a86=sqrt(a86);
  a82=(a82-a86);
  a82=(-a82);
  a90=casadi_fmax(a3,a82);
  a91=casadi_sq(a33);
  a92=arg[0]? arg[0][5] : 0;
  a93=(a64+a92);
  a94=cos(a93);
  a95=(a94*a7);
  a95=(a95-a24);
  a96=casadi_sq(a95);
  a97=sin(a93);
  a98=(a97*a7);
  a98=(a98-a24);
  a99=casadi_sq(a98);
  a96=(a96+a99);
  a99=(a91*a96);
  a100=(a84-a28);
  a101=casadi_sq(a100);
  a102=(a88-a21);
  a103=casadi_sq(a102);
  a101=(a101+a103);
  a103=(a101*a96);
  a104=cos(a93);
  a105=(a104*a7);
  a105=(a105-a24);
  a106=(a84-a28);
  a107=(a105*a106);
  a108=sin(a93);
  a109=(a108*a7);
  a109=(a109-a24);
  a110=(a88-a21);
  a111=(a109*a110);
  a107=(a107+a111);
  a111=casadi_sq(a107);
  a103=(a103-a111);
  a99=(a99-a103);
  a103=casadi_fmax(a3,a99);
  a111=(a90*a103);
  a51=(a51+a111);
  a22=(a22+a52);
  a111=cos(a22);
  a111=(a111*a24);
  a28=(a28+a111);
  a111=(a28-a30);
  a111=casadi_sq(a111);
  a112=sin(a22);
  a112=(a112*a24);
  a21=(a21+a112);
  a112=(a21-a46);
  a112=casadi_sq(a112);
  a111=(a111+a112);
  a111=sqrt(a111);
  a112=cos(a93);
  a113=(a112*a9);
  a113=(a84+a113);
  a114=(a113-a30);
  a115=casadi_sq(a114);
  a116=sin(a93);
  a117=(a116*a9);
  a117=(a88+a117);
  a118=(a117-a46);
  a119=casadi_sq(a118);
  a115=(a115+a119);
  a115=sqrt(a115);
  a111=(a111-a115);
  a111=(-a111);
  a119=casadi_fmax(a3,a111);
  a120=casadi_sq(a33);
  a121=arg[0]? arg[0][7] : 0;
  a122=(a93+a121);
  a123=cos(a122);
  a124=(a123*a9);
  a124=(a124-a24);
  a125=casadi_sq(a124);
  a126=sin(a122);
  a127=(a126*a9);
  a127=(a127-a24);
  a128=casadi_sq(a127);
  a125=(a125+a128);
  a128=(a120*a125);
  a129=(a113-a28);
  a130=casadi_sq(a129);
  a131=(a117-a21);
  a132=casadi_sq(a131);
  a130=(a130+a132);
  a132=(a130*a125);
  a133=cos(a122);
  a134=(a133*a9);
  a134=(a134-a24);
  a135=(a113-a28);
  a136=(a134*a135);
  a137=sin(a122);
  a138=(a137*a9);
  a138=(a138-a24);
  a139=(a117-a21);
  a140=(a138*a139);
  a136=(a136+a140);
  a140=casadi_sq(a136);
  a132=(a132-a140);
  a128=(a128-a132);
  a132=casadi_fmax(a3,a128);
  a140=(a119*a132);
  a51=(a51+a140);
  a22=(a22+a52);
  a52=cos(a22);
  a52=(a52*a24);
  a28=(a28+a52);
  a52=(a28-a30);
  a52=casadi_sq(a52);
  a22=sin(a22);
  a22=(a22*a24);
  a21=(a21+a22);
  a22=(a21-a46);
  a22=casadi_sq(a22);
  a52=(a52+a22);
  a52=sqrt(a52);
  a22=cos(a122);
  a140=(a22*a11);
  a140=(a113+a140);
  a141=(a140-a30);
  a142=casadi_sq(a141);
  a143=sin(a122);
  a144=(a143*a11);
  a144=(a117+a144);
  a145=(a144-a46);
  a146=casadi_sq(a145);
  a142=(a142+a146);
  a142=sqrt(a142);
  a52=(a52-a142);
  a52=(-a52);
  a146=casadi_fmax(a3,a52);
  a33=casadi_sq(a33);
  a147=arg[0]? arg[0][9] : 0;
  a148=(a122+a147);
  a149=cos(a148);
  a150=(a149*a11);
  a150=(a150-a24);
  a151=casadi_sq(a150);
  a152=sin(a148);
  a153=(a152*a11);
  a153=(a153-a24);
  a154=casadi_sq(a153);
  a151=(a151+a154);
  a154=(a33*a151);
  a155=(a140-a28);
  a156=casadi_sq(a155);
  a157=(a144-a21);
  a158=casadi_sq(a157);
  a156=(a156+a158);
  a158=(a156*a151);
  a159=cos(a148);
  a160=(a159*a11);
  a160=(a160-a24);
  a28=(a140-a28);
  a161=(a160*a28);
  a162=sin(a148);
  a163=(a162*a11);
  a163=(a163-a24);
  a21=(a144-a21);
  a24=(a163*a21);
  a161=(a161+a24);
  a24=casadi_sq(a161);
  a158=(a158-a24);
  a154=(a154-a158);
  a158=casadi_fmax(a3,a154);
  a24=(a146*a158);
  a51=(a51+a24);
  a24=arg[1]? arg[1][1] : 0;
  a24=(a24/a13);
  a51=(a51+a24);
  a51=(a51+a51);
  a2=(a2*a13);
  a51=(a51*a2);
  a50=(a50*a51);
  a44=(a44*a50);
  a32=(a32*a44);
  a23=(a23*a32);
  a18=(a18*a23);
  a4=(a4+a18);
  a29=(a29*a32);
  a25=(a25*a29);
  a4=(a4+a25);
  a39=(a39+a39);
  a34=(a34*a44);
  a42=(a42*a44);
  a34=(a34-a42);
  a39=(a39*a34);
  a38=(a38*a39);
  a4=(a4+a38);
  a36=(a36+a36);
  a36=(a36*a34);
  a35=(a35*a36);
  a4=(a4+a35);
  a31=(a31*a32);
  a43=(a43+a43);
  a37=(a37*a44);
  a43=(a43*a37);
  a31=(a31-a43);
  a49=(a49+a49);
  a45=(a45<=a3);
  a45=(!a45);
  a40=(a40*a51);
  a45=(a45*a40);
  a48=(a48+a48);
  a45=(a45/a48);
  a49=(a49*a45);
  a31=(a31+a49);
  a78=(a78+a78);
  a70=(a70<=a3);
  a70=(!a70);
  a61=(a61*a51);
  a70=(a70*a61);
  a78=(a78*a70);
  a80=(a80*a78);
  a73=(a73+a73);
  a67=(a67*a70);
  a73=(a73*a67);
  a80=(a80-a73);
  a60=(a60+a60);
  a53=(a53<=a3);
  a53=(!a53);
  a74=(a74*a51);
  a53=(a53*a74);
  a57=(a57+a57);
  a53=(a53/a57);
  a60=(a60*a53);
  a80=(a80+a60);
  a107=(a107+a107);
  a99=(a99<=a3);
  a99=(!a99);
  a90=(a90*a51);
  a99=(a99*a90);
  a107=(a107*a99);
  a109=(a109*a107);
  a102=(a102+a102);
  a96=(a96*a99);
  a102=(a102*a96);
  a109=(a109-a102);
  a89=(a89+a89);
  a82=(a82<=a3);
  a82=(!a82);
  a103=(a103*a51);
  a82=(a82*a103);
  a86=(a86+a86);
  a82=(a82/a86);
  a89=(a89*a82);
  a109=(a109+a89);
  a136=(a136+a136);
  a128=(a128<=a3);
  a128=(!a128);
  a119=(a119*a51);
  a128=(a128*a119);
  a136=(a136*a128);
  a138=(a138*a136);
  a131=(a131+a131);
  a125=(a125*a128);
  a131=(a131*a125);
  a138=(a138-a131);
  a118=(a118+a118);
  a111=(a111<=a3);
  a111=(!a111);
  a132=(a132*a51);
  a111=(a111*a132);
  a115=(a115+a115);
  a111=(a111/a115);
  a118=(a118*a111);
  a138=(a138+a118);
  a161=(a161+a161);
  a154=(a154<=a3);
  a154=(!a154);
  a146=(a146*a51);
  a154=(a154*a146);
  a161=(a161*a154);
  a163=(a163*a161);
  a157=(a157+a157);
  a151=(a151*a154);
  a157=(a157*a151);
  a163=(a163-a157);
  a145=(a145+a145);
  a52=(a52<=a3);
  a52=(!a52);
  a158=(a158*a51);
  a52=(a52*a158);
  a142=(a142+a142);
  a52=(a52/a142);
  a145=(a145*a52);
  a163=(a163+a145);
  a145=10.;
  a144=(a144-a46);
  a144=(a144+a144);
  a144=(a145*a144);
  a163=(a163+a144);
  a138=(a138+a163);
  a117=(a117-a46);
  a117=(a117+a117);
  a117=(a145*a117);
  a138=(a138+a117);
  a109=(a109+a138);
  a88=(a88-a46);
  a88=(a88+a88);
  a88=(a145*a88);
  a109=(a109+a88);
  a80=(a80+a109);
  a59=(a59-a46);
  a59=(a59+a59);
  a59=(a145*a59);
  a80=(a80+a59);
  a31=(a31+a80);
  a19=(a19-a46);
  a19=(a19+a19);
  a19=(a145*a19);
  a31=(a31+a19);
  a20=(a20*a31);
  a4=(a4+a20);
  a26=(a26*a32);
  a41=(a41+a41);
  a41=(a41*a37);
  a26=(a26-a41);
  a47=(a47+a47);
  a47=(a47*a45);
  a26=(a26+a47);
  a76=(a76*a78);
  a71=(a71+a71);
  a71=(a71*a67);
  a76=(a76-a71);
  a56=(a56+a56);
  a56=(a56*a53);
  a76=(a76+a56);
  a105=(a105*a107);
  a100=(a100+a100);
  a100=(a100*a96);
  a105=(a105-a100);
  a85=(a85+a85);
  a85=(a85*a82);
  a105=(a105+a85);
  a134=(a134*a136);
  a129=(a129+a129);
  a129=(a129*a125);
  a134=(a134-a129);
  a114=(a114+a114);
  a114=(a114*a111);
  a134=(a134+a114);
  a160=(a160*a161);
  a155=(a155+a155);
  a155=(a155*a151);
  a160=(a160-a155);
  a141=(a141+a141);
  a141=(a141*a52);
  a160=(a160+a141);
  a140=(a140-a30);
  a140=(a140+a140);
  a140=(a145*a140);
  a160=(a160+a140);
  a134=(a134+a160);
  a113=(a113-a30);
  a113=(a113+a113);
  a113=(a145*a113);
  a134=(a134+a113);
  a105=(a105+a134);
  a84=(a84-a30);
  a84=(a84+a84);
  a84=(a145*a84);
  a105=(a105+a84);
  a76=(a76+a105);
  a55=(a55-a30);
  a55=(a55+a55);
  a55=(a145*a55);
  a76=(a76+a55);
  a26=(a26+a76);
  a27=(a27-a30);
  a27=(a27+a27);
  a145=(a145*a27);
  a26=(a26+a145);
  a15=(a15*a26);
  a4=(a4+a15);
  if (res[0]!=0) res[0][0]=a4;
  a4=(a16-a63);
  a15=5.0000000000000003e-002;
  a4=(a4-a15);
  a26=(a4<=a3);
  a26=(!a26);
  a145=arg[2]? arg[2][4] : 0;
  a27=(a145-a16);
  a27=(a27-a15);
  a30=casadi_fmax(a3,a27);
  a4=casadi_fmax(a3,a4);
  a30=(a30+a4);
  a4=(a63-a92);
  a4=(a4-a15);
  a55=casadi_fmax(a3,a4);
  a30=(a30+a55);
  a55=(a92-a121);
  a55=(a55-a15);
  a84=casadi_fmax(a3,a55);
  a30=(a30+a84);
  a84=(a121-a147);
  a84=(a84-a15);
  a113=casadi_fmax(a3,a84);
  a30=(a30+a113);
  a30=(a30+a30);
  a30=(a30*a14);
  a26=(a26*a30);
  a27=(a27<=a3);
  a27=(!a27);
  a27=(a27*a30);
  a27=(a26-a27);
  a113=(a63-a16);
  a113=(a113-a15);
  a140=(a113<=a3);
  a140=(!a140);
  a16=(a16-a145);
  a16=(a16-a15);
  a145=casadi_fmax(a3,a16);
  a113=casadi_fmax(a3,a113);
  a145=(a145+a113);
  a63=(a92-a63);
  a63=(a63-a15);
  a113=casadi_fmax(a3,a63);
  a145=(a145+a113);
  a92=(a121-a92);
  a92=(a92-a15);
  a113=casadi_fmax(a3,a92);
  a145=(a145+a113);
  a147=(a147-a121);
  a147=(a147-a15);
  a15=casadi_fmax(a3,a147);
  a145=(a145+a15);
  a145=(a145+a145);
  a145=(a145*a14);
  a140=(a140*a145);
  a27=(a27-a140);
  a16=(a16<=a3);
  a16=(!a16);
  a16=(a16*a145);
  a27=(a27+a16);
  a16=cos(a17);
  a23=(a0*a23);
  a16=(a16*a23);
  a23=sin(a17);
  a29=(a0*a29);
  a23=(a23*a29);
  a16=(a16-a23);
  a23=cos(a17);
  a39=(a0*a39);
  a23=(a23*a39);
  a16=(a16+a23);
  a23=sin(a17);
  a0=(a0*a36);
  a23=(a23*a0);
  a16=(a16-a23);
  a23=cos(a64);
  a81=(a81*a78);
  a0=(a5*a81);
  a23=(a23*a0);
  a0=sin(a64);
  a77=(a77*a78);
  a78=(a5*a77);
  a0=(a0*a78);
  a23=(a23-a0);
  a0=cos(a64);
  a69=(a69+a69);
  a62=(a62*a70);
  a72=(a72*a70);
  a62=(a62-a72);
  a69=(a69*a62);
  a72=(a5*a69);
  a0=(a0*a72);
  a23=(a23+a0);
  a0=sin(a64);
  a66=(a66+a66);
  a66=(a66*a62);
  a62=(a5*a66);
  a0=(a0*a62);
  a23=(a23-a0);
  a0=cos(a93);
  a110=(a110*a107);
  a62=(a7*a110);
  a0=(a0*a62);
  a62=sin(a93);
  a106=(a106*a107);
  a107=(a7*a106);
  a62=(a62*a107);
  a0=(a0-a62);
  a62=cos(a93);
  a98=(a98+a98);
  a91=(a91*a99);
  a101=(a101*a99);
  a91=(a91-a101);
  a98=(a98*a91);
  a101=(a7*a98);
  a62=(a62*a101);
  a0=(a0+a62);
  a62=sin(a93);
  a95=(a95+a95);
  a95=(a95*a91);
  a91=(a7*a95);
  a62=(a62*a91);
  a0=(a0-a62);
  a62=cos(a148);
  a21=(a21*a161);
  a91=(a11*a21);
  a62=(a62*a91);
  a91=sin(a148);
  a28=(a28*a161);
  a161=(a11*a28);
  a91=(a91*a161);
  a62=(a62-a91);
  a91=cos(a148);
  a153=(a153+a153);
  a33=(a33*a154);
  a156=(a156*a154);
  a33=(a33-a156);
  a153=(a153*a33);
  a156=(a11*a153);
  a91=(a91*a156);
  a62=(a62+a91);
  a148=sin(a148);
  a150=(a150+a150);
  a150=(a150*a33);
  a33=(a11*a150);
  a148=(a148*a33);
  a62=(a62-a148);
  a148=cos(a122);
  a139=(a139*a136);
  a33=(a9*a139);
  a148=(a148*a33);
  a148=(a62+a148);
  a33=sin(a122);
  a135=(a135*a136);
  a136=(a9*a135);
  a33=(a33*a136);
  a148=(a148-a33);
  a33=cos(a122);
  a127=(a127+a127);
  a120=(a120*a128);
  a130=(a130*a128);
  a120=(a120-a130);
  a127=(a127*a120);
  a130=(a9*a127);
  a33=(a33*a130);
  a148=(a148+a33);
  a33=sin(a122);
  a124=(a124+a124);
  a124=(a124*a120);
  a120=(a9*a124);
  a33=(a33*a120);
  a148=(a148-a33);
  a33=cos(a122);
  a120=(a11*a163);
  a33=(a33*a120);
  a148=(a148+a33);
  a122=sin(a122);
  a11=(a11*a160);
  a122=(a122*a11);
  a148=(a148-a122);
  a0=(a0+a148);
  a122=cos(a93);
  a11=(a9*a138);
  a122=(a122*a11);
  a0=(a0+a122);
  a93=sin(a93);
  a9=(a9*a134);
  a93=(a93*a9);
  a0=(a0-a93);
  a23=(a23+a0);
  a93=cos(a64);
  a9=(a7*a109);
  a93=(a93*a9);
  a23=(a23+a93);
  a64=sin(a64);
  a7=(a7*a105);
  a64=(a64*a7);
  a23=(a23-a64);
  a16=(a16+a23);
  a64=cos(a17);
  a7=(a5*a80);
  a64=(a64*a7);
  a16=(a16+a64);
  a17=sin(a17);
  a5=(a5*a76);
  a17=(a17*a5);
  a16=(a16-a17);
  a27=(a27+a16);
  if (res[0]!=0) res[0][1]=a27;
  a8=(a8<=a3);
  a8=(!a8);
  a8=(a8*a1);
  a6=(a6-a8);
  a79=(a79*a81);
  a6=(a6+a79);
  a75=(a75*a77);
  a6=(a6+a75);
  a68=(a68*a69);
  a6=(a6+a68);
  a65=(a65*a66);
  a6=(a6+a65);
  a58=(a58*a80);
  a6=(a6+a58);
  a54=(a54*a76);
  a6=(a6+a54);
  if (res[0]!=0) res[0][2]=a6;
  a4=(a4<=a3);
  a4=(!a4);
  a4=(a4*a30);
  a26=(a4-a26);
  a63=(a63<=a3);
  a63=(!a63);
  a63=(a63*a145);
  a26=(a26-a63);
  a26=(a26+a140);
  a26=(a26+a23);
  if (res[0]!=0) res[0][3]=a26;
  a10=(a10<=a3);
  a10=(!a10);
  a10=(a10*a1);
  a8=(a8-a10);
  a108=(a108*a110);
  a8=(a8+a108);
  a104=(a104*a106);
  a8=(a8+a104);
  a97=(a97*a98);
  a8=(a8+a97);
  a94=(a94*a95);
  a8=(a8+a94);
  a87=(a87*a109);
  a8=(a8+a87);
  a83=(a83*a105);
  a8=(a8+a83);
  if (res[0]!=0) res[0][4]=a8;
  a55=(a55<=a3);
  a55=(!a55);
  a55=(a55*a30);
  a4=(a55-a4);
  a92=(a92<=a3);
  a92=(!a92);
  a92=(a92*a145);
  a4=(a4-a92);
  a4=(a4+a63);
  a4=(a4+a0);
  if (res[0]!=0) res[0][5]=a4;
  a12=(a12<=a3);
  a12=(!a12);
  a12=(a12*a1);
  a10=(a10-a12);
  a137=(a137*a139);
  a10=(a10+a137);
  a133=(a133*a135);
  a10=(a10+a133);
  a126=(a126*a127);
  a10=(a10+a126);
  a123=(a123*a124);
  a10=(a10+a123);
  a116=(a116*a138);
  a10=(a10+a116);
  a112=(a112*a134);
  a10=(a10+a112);
  if (res[0]!=0) res[0][6]=a10;
  a84=(a84<=a3);
  a84=(!a84);
  a84=(a84*a30);
  a55=(a84-a55);
  a147=(a147<=a3);
  a147=(!a147);
  a147=(a147*a145);
  a55=(a55-a147);
  a55=(a55+a92);
  a55=(a55+a148);
  if (res[0]!=0) res[0][7]=a55;
  a162=(a162*a21);
  a12=(a12+a162);
  a159=(a159*a28);
  a12=(a12+a159);
  a152=(a152*a153);
  a12=(a12+a152);
  a149=(a149*a150);
  a12=(a12+a149);
  a143=(a143*a163);
  a12=(a12+a143);
  a22=(a22*a160);
  a12=(a12+a22);
  if (res[0]!=0) res[0][8]=a12;
  a147=(a147-a84);
  a147=(a147+a62);
  if (res[0]!=0) res[0][9]=a147;
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_InkASXGEZkQWNKKgVChq(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int grad_phi_InkASXGEZkQWNKKgVChq_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int grad_phi_InkASXGEZkQWNKKgVChq_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_InkASXGEZkQWNKKgVChq_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int grad_phi_InkASXGEZkQWNKKgVChq_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void grad_phi_InkASXGEZkQWNKKgVChq_release(int mem) {
}

CASADI_SYMBOL_EXPORT void grad_phi_InkASXGEZkQWNKKgVChq_incref(void) {
}

CASADI_SYMBOL_EXPORT void grad_phi_InkASXGEZkQWNKKgVChq_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_InkASXGEZkQWNKKgVChq_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int grad_phi_InkASXGEZkQWNKKgVChq_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real grad_phi_InkASXGEZkQWNKKgVChq_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_InkASXGEZkQWNKKgVChq_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* grad_phi_InkASXGEZkQWNKKgVChq_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_InkASXGEZkQWNKKgVChq_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* grad_phi_InkASXGEZkQWNKKgVChq_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int grad_phi_InkASXGEZkQWNKKgVChq_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
