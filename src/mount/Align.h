// -----------------------------------------------------------------------------------
// GEOMETRIC ALIGN FOR ALT/AZM AND EQ MOUNTS
//
// by Howard Dutton
//
// Copyright (C) 2012 to 2020 Howard Dutton
//

#pragma once

//--------------------------------------------------------------------------------------------------
// telescope mount control
#include <Arduino.h>
#include "../../Constants.h"
#include "../../Config.h"
#include "../../ConfigX.h"
#include "../HAL/HAL.h"
#include "../lib/nv/NV.h"
extern NVS nv;
#include "../pinmaps/Models.h"
#include "../debug/Debug.h"
#include "../tasks/OnTask.h"
extern Tasks tasks;

#include "../commands/ProcessCmds.h"

// -----------------------------------------------------------------------------------
// ADVANCED GEOMETRIC ALIGN FOR EQUATORIAL MOUNTS (GOTO ASSIST)

typedef struct {
  double ha;
  double dec;
  double alt;
  double azm;
  int side;
} align_coord2_t;

class TGeoAlign
{
  public:
    double ax1Cor;
    double ax2Cor;
    double altCor;
    double azmCor;
    double doCor;
    double pdCor;
    double dfCor;
    double tfCor;
    align_coord2_t mount[9];
    align_coord2_t actual[9];
    align_coord2_t delta[9];

    void init();
    void readCoe();
    void writeCoe();
    bool isReady();
    CommandError addStar(int I, int N, double RA, double Dec);
    void equToInstr(double HA, double Dec, double *HA1, double *Dec1, int PierSide);
    void instrToEqu(double HA, double Dec, double *HA1, double *Dec1, int PierSide);
    void autoModel(int n);
    void model(int n);

  private:
    bool geo_ready;
    double avgDec;
    double avgHA;

    double lat,cosLat,sinLat;

    long num,l;
    long Ff,Df;
    double best_deo, best_pd, best_pz, best_pe, best_ohw, best_odw, best_ohe, best_ode, best_tf, best_df, best_ff;
    double h1,d1;
    double avg_ha,avg_dec;
    double dist,sumd,rms;
    double best_dist;
    double ohe,ode,ohw,odw,dh;
    double sd,sh,sum1;
    double max_dist;

    void correct(double ha, double dec, double pierSide, double sf, double _deo, double _pd, double _pz, double _pe, double _da, double _ff, double _tf, double *h1, double *d1);
    void do_search(double sf, int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8, int p9);
};

TGeoAlign AlignE;

byte alignNumStars = 0;
byte alignThisStar = 0;

// checks to see if an alignment is active
bool alignActive() {
  return (alignNumStars > 0) && (alignThisStar <= alignNumStars);
}

// adds an alignment star, returns true on success
CommandError alignStar() {
  // after last star turn meridian flips off when align is done
  if ((alignNumStars == alignThisStar) && (meridianFlip == MeridianFlipAlign)) meridianFlip=MeridianFlipNever;

  if (alignThisStar <= alignNumStars) {
    CommandError e;
    if (mountType == ALTAZM) e=AlignH.addStar(alignThisStar,alignNumStars,newTargetRA,newTargetDec); else e=AlignE.addStar(alignThisStar,alignNumStars,newTargetRA,newTargetDec);
    if (e == CE_NONE) alignThisStar++; else return e;
  } else return CE_PARAM_RANGE;

  return CE_NONE;
}
