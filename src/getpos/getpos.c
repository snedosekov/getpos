/*------------------------------------------------------------------------------
* rtkrcv.c : rtk-gps/gnss receiver console ap
*
*          Copyright (C) 2009-2015 by T.TAKASU, All rights reserved.
*
* notes   :
*     current version does not support win32 without pthread library
*
* version : $Revision:$ $Date:$
* history : 2009/12/13 1.0  new
*           2010/07/18 1.1  add option -m
*           2010/08/12 1.2  fix bug on ftp/http
*           2011/01/22 1.3  add option misc-proxyaddr,misc-fswapmargin
*           2011/08/19 1.4  fix bug on size of arg solopt arg for rtksvrstart()
*           2012/11/03 1.5  fix bug on setting output format
*           2013/06/30 1.6  add "nvs" option for inpstr*-format
*           2014/02/10 1.7  fix bug on printing obs data
*                           add print of status, glonass nav data
*                           ignore SIGHUP
*           2014/04/27 1.8  add "binex" option for inpstr*-format
*           2014/08/10 1.9  fix cpu overload with abnormal telnet shutdown
*           2014/08/26 1.10 support input format "rt17"
*                           change file paths of solution status and debug trace
*           2015/01/10 1.11 add line editting and command history
*                           separate codes for virtual console to vt.c
*-----------------------------------------------------------------------------*/
#include <signal.h>
#include <unistd.h>
#include "rtklib.h"
#include "vt.h"

static const char rcsid[]="$Id:$";

#define PRGNAME     "getpos"            /* program name */
#define CMDPROMPT   "getpos> "          /* command prompt */
#define MAXARG      10                  /* max number of args in a command */
#define MAXCMD      256                 /* max length of a command */
#define MAXSTR      1024                /* max length of a stream */
#define MAXRCVCMD   4096                /* max receiver command */
#define OPTSDIR     "."                 /* default config directory */
#define OPTSFILE    "getpos.conf"       /* default config file */
#define NAVIFILE    "getpos.nav"        /* navigation save file */
#define STATFILE    "getpos_%Y%m%d%h%M.stat"  /* solution status file */
#define TRACEFILE   "getpos_%Y%m%d%h%M.trace" /* debug trace file */
#define INTKEEPALIVE 1000               /* keep alive interval (ms) */

#define ESC_CLEAR   "\033[2J"           /* ansi/vt100: erase screen */
#define ESC_RESET   "\033[0m"           /* ansi/vt100: reset attribute */
#define ESC_BOLD    "\033[1m"           /* ansi/vt100: bold */

#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

/* function prototypes -------------------------------------------------------*/
extern FILE *popen(const char *, const char *);
extern int pclose(FILE *);

/* global variables ----------------------------------------------------------*/
static rtksvr_t svr;                    /* rtk server struct */
static stream_t moni;                   /* monitor stream */

static int intflg       =0;             /* interrupt flag (2:shtdown) */

static char passwd[MAXSTR]="admin";     /* login password */
static int timetype     =0;             /* time format (0:gpst,1:utc,2:jst,3:tow) */
static int soltype      =0;             /* sol format (0:dms,1:deg,2:xyz,3:enu,4:pyl) */
static int solflag      =2;             /* sol flag (1:std+2:age/ratio/ns) */
static int strtype[]={                  /* stream types */
    STR_SERIAL,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE
};
static char strpath[8][MAXSTR]={""};    /* stream paths */
static int strfmt[]={                   /* stream formats */
    STRFMT_UBX,STRFMT_RTCM3,STRFMT_SP3,SOLF_LLH,SOLF_NMEA
};
static int svrcycle     =10;            /* server cycle (ms) */
static int timeout      =10000;         /* timeout time (ms) */
static int reconnect    =10000;         /* reconnect interval (ms) */
static int nmeacycle    =5000;          /* nmea request cycle (ms) */
static int buffsize     =32768;         /* input buffer size (bytes) */
static int navmsgsel    =0;             /* navigation mesaage select */
static char proxyaddr[256]="";          /* http/ntrip proxy */
static int nmeareq      =0;             /* nmea request type (0:off,1:lat/lon,2:single) */
static double nmeapos[] ={0,0};         /* nmea position (lat/lon) (deg) */
static char rcvcmds[3][MAXSTR]={""};    /* receiver commands files */
static char startcmd[MAXSTR]="";        /* start command */
static char stopcmd [MAXSTR]="";        /* stop command */
static int modflgr[256] ={0};           /* modified flags of receiver options */
static int modflgs[256] ={0};           /* modified flags of system options */
static int moniport     =0;             /* monitor port */
static int keepalive    =0;             /* keep alive flag */
static int fswapmargin  =30;            /* file swap margin (s) */

static prcopt_t prcopt;                 /* processing options */
static solopt_t solopt[2]={{0}};        /* solution options */
static filopt_t filopt  ={""};          /* file options */



/* receiver options table ----------------------------------------------------*/
#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,10:nvs,11:binex,12:rt17,15:sp3"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea"
#define MSGOPT  "0:all,1:rover,2:base,3:corr"

static opt_t rcvopts[]={
    {"console-passwd",  2,  (void *)passwd,              ""     },
    {"console-timetype",3,  (void *)&timetype,           TIMOPT },
    {"console-soltype", 3,  (void *)&soltype,            CONOPT },
    {"console-solflag", 0,  (void *)&solflag,            FLGOPT },
    
    {"inpstr1-type",    3,  (void *)&strtype[0],         ISTOPT },
    {"inpstr2-type",    3,  (void *)&strtype[1],         ISTOPT },
    {"inpstr3-type",    3,  (void *)&strtype[2],         ISTOPT },
    {"inpstr1-path",    2,  (void *)strpath [0],         ""     },
    {"inpstr2-path",    2,  (void *)strpath [1],         ""     },
    {"inpstr3-path",    2,  (void *)strpath [2],         ""     },
    {"inpstr1-format",  3,  (void *)&strfmt [0],         FMTOPT },
    {"inpstr2-format",  3,  (void *)&strfmt [1],         FMTOPT },
    {"inpstr3-format",  3,  (void *)&strfmt [2],         FMTOPT },
    {"inpstr2-nmeareq", 3,  (void *)&nmeareq,            NMEOPT },
    {"inpstr2-nmealat", 1,  (void *)&nmeapos[0],         "deg"  },
    {"inpstr2-nmealon", 1,  (void *)&nmeapos[1],         "deg"  },
    {"outstr1-type",    3,  (void *)&strtype[3],         OSTOPT },
    {"outstr2-type",    3,  (void *)&strtype[4],         OSTOPT },
    {"outstr1-path",    2,  (void *)strpath [3],         ""     },
    {"outstr2-path",    2,  (void *)strpath [4],         ""     },
    {"outstr1-format",  3,  (void *)&strfmt [3],         SOLOPT },
    {"outstr2-format",  3,  (void *)&strfmt [4],         SOLOPT },
    {"logstr1-type",    3,  (void *)&strtype[5],         OSTOPT },
    {"logstr2-type",    3,  (void *)&strtype[6],         OSTOPT },
    {"logstr3-type",    3,  (void *)&strtype[7],         OSTOPT },
    {"logstr1-path",    2,  (void *)strpath [5],         ""     },
    {"logstr2-path",    2,  (void *)strpath [6],         ""     },
    {"logstr3-path",    2,  (void *)strpath [7],         ""     },
    
    {"misc-svrcycle",   0,  (void *)&svrcycle,           "ms"   },
    {"misc-timeout",    0,  (void *)&timeout,            "ms"   },
    {"misc-reconnect",  0,  (void *)&reconnect,          "ms"   },
    {"misc-nmeacycle",  0,  (void *)&nmeacycle,          "ms"   },
    {"misc-buffsize",   0,  (void *)&buffsize,           "bytes"},
    {"misc-navmsgsel",  3,  (void *)&navmsgsel,          MSGOPT },
    {"misc-proxyaddr",  2,  (void *)proxyaddr,           ""     },
    {"misc-fswapmargin",0,  (void *)&fswapmargin,        "s"    },
    
    {"misc-startcmd",   2,  (void *)startcmd,            ""     },
    {"misc-stopcmd",    2,  (void *)stopcmd,             ""     },
    
    {"file-cmdfile1",   2,  (void *)rcvcmds[0],          ""     },
    {"file-cmdfile2",   2,  (void *)rcvcmds[1],          ""     },
    {"file-cmdfile3",   2,  (void *)rcvcmds[2],          ""     },
    
    {"",0,NULL,""}
};
/* external stop signal ------------------------------------------------------*/
static void sigshut(int sig)
{
    trace(3,"sigshut: sig=%d\n",sig);
    intflg=1;
}

/* thread to send keep alive for monitor port --------------------------------*/
static void *sendkeepalive(void *arg)
{
    trace(3,"sendkeepalive: start\n");
    
    while (keepalive) {
        strwrite(&moni,(unsigned char *)"\r",1);
        sleepms(INTKEEPALIVE);
    }
    trace(3,"sendkeepalive: stop\n");
    return NULL;
}
/* open monitor port ---------------------------------------------------------*/
static int openmoni(int port)
{
    pthread_t thread;
    char path[64];
    
    trace(3,"openmomi: port=%d\n",port);
    
    sprintf(path,":%d",port);
    if (!stropen(&moni,STR_TCPSVR,STR_MODE_RW,path)) return 0;
    strsettimeout(&moni,timeout,reconnect);
    keepalive=1;
    pthread_create(&thread,NULL,sendkeepalive,NULL);
    return 1;
}
/* close monitor port --------------------------------------------------------*/
static void closemoni(void)
{
    trace(3,"closemoni:\n");
    keepalive=0;
    
    /* send disconnect message */
    strwrite(&moni,(unsigned char *)MSG_DISCONN,strlen(MSG_DISCONN));
    
    /* wait fin from clients */
    sleepms(1000);
    
    strclose(&moni);
}

/* read receiver commands ----------------------------------------------------*/
static int readcmd(const char *file, char *cmd, int type)
{
    FILE *fp;
    char buff[MAXSTR],*p=cmd;
    int i=0;
    
    trace(3,"readcmd: file=%s\n",file);
    
    if (!(fp=fopen(file,"r"))) return 0;
    
    while (fgets(buff,sizeof(buff),fp)) {
        if (*buff=='@') i=1;
        else if (i==type&&p+strlen(buff)+1<cmd+MAXRCVCMD) {
            p+=sprintf(p,"%s",buff);
        }
    }
    fclose(fp);
    return 1;
}
/* read antenna file ---------------------------------------------------------*/
static void readant( prcopt_t *opt, nav_t *nav)
{
    const pcv_t pcv0={0};
    pcvs_t pcvr={0},pcvs={0};
    pcv_t *pcv;
    gtime_t time=timeget();
    int i;
    
    trace(3,"readant:\n");
    
    opt->pcvr[0]=opt->pcvr[1]=pcv0;
    if (!*filopt.rcvantp) return;
    
    if (readpcv(filopt.rcvantp,&pcvr)) {
        for (i=0;i<2;i++) {
            if (!*opt->anttype[i]) continue;
            if (!(pcv=searchpcv(0,opt->anttype[i],time,&pcvr))) {
                printf("no antenna %s in %s",opt->anttype[i],filopt.rcvantp);
                continue;
            }
            opt->pcvr[i]=*pcv;
        }
    }
    else printf("antenna file open error %s",filopt.rcvantp);
    
    if (readpcv(filopt.satantp,&pcvs)) {
        for (i=0;i<MAXSAT;i++) {
            if (!(pcv=searchpcv(i+1,"",time,&pcvs))) continue;
            nav->pcvs[i]=*pcv;
        }
    }
    else printf("antenna file open error %s",filopt.satantp);
    
    free(pcvr.pcv); free(pcvs.pcv);
}
/* start rtk server ----------------------------------------------------------*/
static int startsvr()
{
    double pos[3],npos[3];
    char s[3][MAXRCVCMD]={"","",""},*cmds[]={NULL,NULL,NULL};
    char *ropts[]={"","",""};
    char *paths[]={
        strpath[0],strpath[1],strpath[2],strpath[3],strpath[4],strpath[5],
        strpath[6],strpath[7]
    };
    int i,ret,stropt[8]={0};
    
    trace(3,"startsvr:\n");
    
    /* read start commads from command files */
    for (i=0;i<3;i++) {
        if (!*rcvcmds[i]) continue;
        if (!readcmd(rcvcmds[i],s[i],0)) {
            printf("no command file: %s\n",rcvcmds[i]);
        }
        else cmds[i]=s[i];
    }
    /* confirm overwrite */
   /* for (i=3;i<8;i++) {
        if (strtype[i]==STR_FILE&&!confwrite(strpath[i])) return 0;
    }*/
    if (prcopt.refpos==4) { /* rtcm */
        for (i=0;i<3;i++) prcopt.rb[i]=0.0;
    }
    pos[0]=nmeapos[0]*D2R;
    pos[1]=nmeapos[1]*D2R;
    pos[2]=0.0;
    pos2ecef(pos,npos);
    
    /* read antenna file */
    readant(&prcopt,&svr.nav);
    
    /* open geoid data file */
    if (solopt[0].geoid>0&&!opengeoid(solopt[0].geoid,filopt.geoid)) {
        trace(2,"geoid data open error: %s\n",filopt.geoid);
        printf("geoid data open error: %s\n",filopt.geoid);
    }
    for (i=0;*rcvopts[i].name;i++) modflgr[i]=0;
    for (i=0;*sysopts[i].name;i++) modflgs[i]=0;
    
    /* set stream options */
    stropt[0]=timeout;
    stropt[1]=reconnect;
    stropt[2]=1000;
    stropt[3]=buffsize;
    stropt[4]=fswapmargin;
    strsetopt(stropt);
    
    if (strfmt[2]==8) strfmt[2]=STRFMT_SP3;
    
    /* set ftp/http directory and proxy */
    strsetdir(filopt.tempdir);
    strsetproxy(proxyaddr);
    
    /* execute start command */
    if (*startcmd&&(ret=system(startcmd))) {
        trace(2,"command exec error: %s (%d)\n",startcmd,ret);
        printf("command exec error: %s (%d)\n",startcmd,ret);
    }
    solopt[0].posf=strfmt[3];
    solopt[1].posf=strfmt[4];
    
    /* start rtk server */
    if (!rtksvrstart(&svr,svrcycle,buffsize,strtype,paths,strfmt,navmsgsel,
                     cmds,ropts,nmeacycle,nmeareq,npos,&prcopt,solopt,&moni)) {
        trace(2,"rtk server start error\n");
        printf("rtk server start error\n");
        return 0;
    }
    return 1;
}
/* stop rtk server -----------------------------------------------------------*/
static void stopsvr()
{
    char s[3][MAXRCVCMD]={"","",""},*cmds[]={NULL,NULL,NULL};
    int i,ret;
    
    trace(3,"stopsvr:\n");
    
    if (!svr.state) return;
    
    /* read stop commads from command files */
    for (i=0;i<3;i++) {
        if (!*rcvcmds[i]) continue;
        if (!readcmd(rcvcmds[i],s[i],1)) {
            printf("no command file: %s\n",rcvcmds[i]);
        }
        else cmds[i]=s[i];
    }
    /* stop rtk server */
    rtksvrstop(&svr,cmds);
    
    /* execute stop command */
    if (*stopcmd&&(ret=system(stopcmd))) {
        trace(2,"command exec error: %s (%d)\n",stopcmd,ret);
        printf("command exec error: %s (%d)\n",stopcmd,ret);
    }
    if (solopt[0].geoid>0) closegeoid();
}



/* print status --------------------------------------------------------------*/
static void prxyz()
{
    rtk_t rtk;
    double pos[3];

    rtksvrlock(&svr);
    rtk=svr.rtk;
    rtksvrunlock(&svr);

    if (norm(rtk.sol.rr,3)>0.0)
    	ecef2pos(rtk.sol.rr,pos);
    else
    	pos[0]=pos[1]=pos[2]=0.0;
    if(pos[0] && pos[1] && pos[2])
    	printf("%-28s: %.8f,%.8f,%.3f\n","pos xyh",
            pos[0]*R2D,pos[1]*R2D,pos[2]);
}


/*-----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
    int i,start=0,port=0,outstat=0,trace=0;
    char *dev="",file[MAXSTR]="";
    
    for (i=1;i<argc;i++) {
        if      (!strcmp(argv[i],"-s")) start=1;
        else if (!strcmp(argv[i],"-p")&&i+1<argc) port=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-m")&&i+1<argc) moniport=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-d")&&i+1<argc) dev=argv[++i];
        else if (!strcmp(argv[i],"-o")&&i+1<argc) strcpy(file,argv[++i]);
        else if (!strcmp(argv[i],"-r")&&i+1<argc) outstat=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-t")&&i+1<argc) trace=atoi(argv[++i]);
        else fprintf(stderr,"Unknown option: %s\n",argv[i]);
    }
    if (trace>0) {
        traceopen(TRACEFILE);
        tracelevel(trace);
    }
    /* initialize rtk server and monitor port */
    rtksvrinit(&svr);
    strinit(&moni);
    
    /* load options file */
    if (!*file) sprintf(file,"%s/%s",OPTSDIR,OPTSFILE);
    
    resetsysopts();
    if (!loadopts(file,rcvopts)||!loadopts(file,sysopts)) {
        fprintf(stderr,"no options file: %s. defaults used\n",file);
    }
    getsysopts(&prcopt,solopt,&filopt);
    
    /* read navigation data */
    if (!readnav(NAVIFILE,&svr.nav)) {
        fprintf(stderr,"no navigation data: %s\n",NAVIFILE);
    }
    if (outstat>0) {
        rtkopenstat(STATFILE,outstat);
    }
    /* open monitor port */
    if (moniport>0&&!openmoni(moniport)) {
        fprintf(stderr,"monitor port open error: %d\n",moniport);
        return -1;
    }
    /* start rtk server */
    if (!startsvr()) return -1;
    
    if (signal(SIGINT, sigshut) == SIG_ERR)
    	  printf("\ncan't catch SIGINT\n");    /* keyboard interrupt */
    signal(SIGTERM,sigshut);    /* external shutdown signal */
    signal(SIGUSR2,sigshut);
    signal(SIGHUP ,SIG_IGN);
    signal(SIGPIPE,SIG_IGN);
    
    while (!intflg) {
        sleep(1);
        prxyz();

    }
    /* stop rtk server */
    stopsvr();
    
    if (moniport>0) closemoni();
    
    if (outstat>0) rtkclosestat();
    
    if (trace>0) traceclose();
    
    /* save navigation data */
    if (!savenav(NAVIFILE,&svr.nav)) {
        fprintf(stderr,"navigation data save error: %s\n",NAVIFILE);
    }
    return 0;
}
