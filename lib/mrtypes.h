#pragma once

/* std C header files */
#include <math.h>     /* for M_PI */
#include <stdint.h>   /* for uint32_t, uint8_t, uint16_t */
#include <sys/time.h> /* for struct timeval */

/* local header files */
#include "mrclient.h"
#include <mecarover/mrlogger/mrlogger.h>

#if __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------*/
#ifndef PI
#define PI    M_PI
#endif

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*---------------------------------------------------------------------*/
#ifndef MIN
#define MIN(x,y)     ( ((x) <= (y)) ?  (x) : (y) )
#endif

#ifndef MAX
#define MAX(x,y)     ( ((x) >= (y)) ?  (x) : (y) )
#endif

#ifndef ABS
#define ABS(x)       ( (x) >= 0 ? (x) : (-(x)) )
#endif

#ifndef RAD
#define RAD(grad)    ( PI*(grad)/180.0 )
#endif

#ifndef GRAD
#define GRAD(rad)    ( 180.0*(rad)/PI )
#endif

// Beschraenkung eines Winkels auf 0 ... 2PI
#ifndef MAX2PI
#define MAX2PI(x) ((x) > (2*PI)?((x) - (2*PI)):((x) < 0?((x) + (2*PI)):x))
#endif

// Beschraenkung eines Winkels auf -PI ... +PI
#ifndef MAXPI
#define MAXPI(x) ((x) > (PI)?((x) - (2*PI)):((x) < -PI?((x) + (2*PI)):x))
#endif

  typedef double FourReal_t[4];

  typedef struct {
    double x;
    double y;
    double theta;
  } Pose_t;

  typedef struct {
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double omega;
  } PoseV_t;

  typedef struct {
    double delta_x;
    double delta_y;
    double delta_theta;
    double delta_phi_e;
  } rks_move_t;

/*-------           Beschreibung des Fahrzeuges                   -----*/

  typedef struct {
    mrc_vehicletype_t type;	/* Fahrzeug-Typ                    */
    double Inkremente;		/* Geberinkremente (4 * Striche)   */
    double Uebersetzung;	/* Getriebeuebersetzung            */
    double OmegaMax;		/* Motornenndrehzahl in U/min      */
//    double Volt;		/* Steuereing. Steller +/-         */
//    double Ink2Volt;		/* Ink/Ta  --> Volt (Dzr)          */
//    double Rad2Volt;		/* rad/s --> Volt                  */
    double Rad2RPM;		/* rad/s --> RPM                   */
    double Ink2Rad;		/* Ink --> rad                     */
    double Laenge;		/* in mm Abstand der Achsen        */
    double Breite;		/* in mm Abstand der Achsen        */
    double LplusBhalbe;		/* in mm 0.5 * (Breite + Laenge)   */
    double Radradius;		/* in mm                           */
    double Rollenradius;	/* in mm                           */
    double VBahnMax;		/* in mm/s                         */
    double VthetaMax;		/* in rad/s                        */
//    double VoltMax;		/* maximale Drehzahl in V          */
    double JoystickBeschl;	/* maximale Jostickbeschleunigung in mm/s/s */
    int SchutzfeldUmschaltung;	/* Soll das Schutzfeld der Lasterscanner vom Interpolator umgeschaltet werden? */
    int HubzylinderVorhanden;	/* Ist auf dem Fahrzeug ein Hubzylinder vorhanden, der gesteuert werden soll? */
    int ArmVorhanden;		/* Ist auf dem Fahrzeug ein Arm vorhanden, der gesteuert werden soll? */
    int BreakDown;		/* Soll das Fahrzeug bei einer Verletzung des Warnfelds stehen bleiben oder langsamer werden? */
//    mrc_mode Modus;		/* Betriebsmodus des Fahrzeugs */
    mrc_stat Stoerung;		/* Art der Stoerung */
    mrc_stat_gyro GyroError;	/* Fehler beim Auslesen des Gyroskops */
//    int PoseUpdate;		/* Zaehler fuer Update der Pose */
    struct {
      double a;       /* Translatorische Bremsbeschleunigung */
      double alpha;   /* Rotatorische Bremsbeschleunigung */
    } bremsbeschl;    /* Bremsbeschleunigungen bei Nothalt */
    double MaxRPM[4];		/* Maximum Umdrehungen pro Minute Motor */
    int UsePipe;		/* Use fds 3 and 4 for communication with mrserver */
    struct {
    bool UseColors;
    mr_logprio_t LogLevel;
    mr_logprio_t ScreenLevel;
    } log;
  } Fahrzeug_t;

/* ------- Datentyp für das Loggen von Laserbeams ------- */
typedef struct {
  struct {
    int16_t x;
    int16_t y;
    double phi;
  } __attribute__ ((packed, aligned(4))) offset;
  uint16_t nr_beams;
  struct {
    double angle;
    uint16_t dist;
  } __attribute__ ((packed, aligned(4))) beams[MRC_BEAMS_MAX];
} mr_datensenke_laserbeams_t;

typedef struct {
  uint16_t nr_beams;
  struct {
    uint16_t angle_z_q14;
    uint32_t dist_mm_q2;
  } __attribute__ ((__packed__)) beams[MRC_BEAMS_MAX];
} mr_datensenke_rplidarbeams_t;

/* ------- Datentyp für das Loggen von Lasersegmente ------- */
typedef struct {
  struct {
    int16_t x;
    int16_t y;
    double phi;
  } __attribute__ ((packed, aligned(4))) offset;
  uint8_t nr_segments;
  struct {
    double angle;
    uint16_t dist;
    int32_t xstart;
    int32_t ystart;
    int32_t xend;
    int32_t yend;
  } __attribute__ ((packed, aligned(4))) segments[MRC_LASERSEGMENTS_MAX];
} mr_datensenke_lasersegments_t;

/* ------- Datentyp für das Loggen von Gyroupdates ------- */

typedef struct {
  double angle_abs;
  double angle_vel;
} mr_datensenke_gyro_t;

/* ------- Datentyp für das Loggen der Encoder ------- */

typedef uint32_t mr_datensenke_encoder_t[4];
typedef uint32_t mr_datensenke_actuator_encoder_t[MRC_ACTUATOR_ID_MAX];

/* ------- Datentyp für das Loggen von Actuatorenpositionen ------- */
typedef mrc_actuator_position_t mr_datensenke_actuator_position_t[MRC_ACTUATOR_ID_MAX];

/* ------- Datentyp für das Loggen von GNSS-SVINFO-Daten ------- */

typedef struct {
  uint8_t numCh;
  struct {
    uint8_t chn;
    uint8_t svid;
    uint8_t flags;
    uint8_t quality;
    uint8_t cno;
    int8_t elev;
    int16_t azim;
    int32_t prRes;
  } ChInfo[UINT8_MAX];
} mr_datensenke_gnss_svinfo_t;

/* ------- Datentyp für das Loggen von Radar-Objekt-Daten ------- */

typedef struct {
  uint8_t NofObjects;
  uint16_t MeasCounter;
  uint8_t InterfaceVersion;
} mr_datensenke_radar_object_status_t;

typedef struct {
  uint8_t ID;
  double DistLong;
  double DistLat;
  double VrelLong;
  double VrelLat;
  uint8_t DynProp;
  double RCS;
} __attribute__ ((packed)) mr_datensenke_radar_object_general_t;

typedef struct {
  uint8_t ID;
  uint8_t DistLong_rms;
  uint8_t VrelLong_rms;
  uint8_t ArelLong_rms;
  uint8_t DistLat_rms;
  uint8_t VrelLat_rms;
  uint8_t ArelLat_rms;
  uint8_t Orientation_rms;
  uint8_t MeasState;
  uint8_t PropOfExist;
} mr_datensenke_radar_object_quality_t;

typedef struct {
  uint8_t ID;
  double ArelLong;
  uint8_t Class;
  double ArelLat;
  double OrientationAngel;
  double Length;
  double Width;
} __attribute__ ((packed)) mr_datensenke_radar_object_extended_t;

typedef struct {
  uint8_t ID;
  uint8_t CollDetRegionBitfield;
} mr_datensenke_radar_object_warning_t;

/*------- Kommunikationsstrukturen fuer Datenaustausch mit Userspace -----*/

/* Version of the protocol implemented here */
#define MR_PROTOVERS 0x00

  typedef enum mr_befehl {
    MR_NOOP,
    MR_SETZEMANUELLRKS,
    MR_SETZEMODUS,
    MR_GETODO,
    MR_FAHREKURS,
    MR_SETZEHUB,
    MR_SETZEPOSE,
    MR_LESESTATUS,
    MR_SETZELOCALIZATIONSTATUS,
    MR_LESELOCALIZATIONSTATUS,
    MR_LESEALLEPOSEN,
    MR_PUSH_SUB,
    MR_PUSH_UNSUB,
    MR_GETGYRO,
    MR_GETRADGESCHW,
    MR_GETLASERDATA,
    MR_GETCONFIGUREDLASER,
    MR_GETVEHICLEPARAMETER,
    MR_SENDESTATUS,
    MR_COURSEFILELIST,
    MR_LOADCOURSEFILE,
    MR_SAVECOURSEFILE,
    MR_DELTECOURSEFILE,
    MR_RESPONSE,
    MR_HELO,
    MR_AUTH,
    MR_PING,
    MR_MOTORCURRENT,
    MR_GETRFIDSCAN,
    MR_GETCONFIGUREDRFID,
    MR_SETINTERPOLMODE,
    MR_GETVECTORMAP,
    MR_VECTORMAPFILELIST,
    MR_GETMOTORDATA,
    MR_GETBATTSTAT,
    MR_RESET_NOTAUSKREIS,
    MR_LANDMARKSFILELIST,
    MR_LOADLANDMARKSFILE,
    MR_SAVELANDMARKSFILE,
    MR_DELTELANDMARKSFILE,
    MR_RFID_EVENT,
    MR_GETUWBSCAN,
    MR_GETCONFIGUREDUWB,
    MR_UWB_EVENT,
    MR_ACTUATOR_GETLIST,
    MR_ACTUATOR_GETSTATUSLIST,
    MR_ACTUATOR_GETSTATUS,
    MR_ACTUATOR_GETDIST,
    MR_ACTUATOR_SETDIST,
    MR_GETBATTSTAT_EXT,
    MR_GETCURINTERPOLSEGNR,
    MR_GETSVCPROGSTATS,
    MR_SVCPROGACTION,
    MR_COLLISION_EVENT,
    MR_DRIVETO_VERTEX,
    MR_GNSS_ECEF_EVENT,
    MR_MODE_EVENT,
    MR_ACTUATOR_REFERENCE,
    MR_ACTUATOR_CONFIG_GET,
    MR_ACTUATOR_CONFIG_SET,
    MR_GNSS_CORNER_UPDATE,
/* Add new elements above this line */
    MR_LASTOPT
  } mr_paketbefehl;

  /**
   * Datatype to specify the push frequency for a certain push packet
   * type to be pushed from the server to a client.
   *
   * This datatype is used for the subscribe and unsubscribe data
   * packets as well as inside the mrservers's own datatypes used
   * to keep record of the current subscriptions of every client.
   */
  typedef struct {
    /** Type of the requested push packet */
    mrc_callback_type type;
    /** Period of time between two push packets from the server */
    struct timeval period;
  } mr_push_t;

  enum mr_odotyp {
    MR_ODO,
    MR_ODO_GYRO,
    MR_ODO_VEL,
    MR_RKS_VEL,
  };

  enum mr_posetyp {
    MR_POSE_ALLE,
    MR_POSE,
    MR_POSE_DELTA,
  };

  typedef union {
    Pose_t koordinaten;
    PoseV_t koordinaten2;
  } mr_odokoord;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    Pose_t geschwindigkeiten;
  } __attribute__ ((__packed__)) mr_setmanuellrks;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_mode modus;
  } __attribute__ ((__packed__)) mr_setze_modus;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    enum mr_odotyp typ;
    mr_odokoord daten;
  } __attribute__ ((__packed__)) mr_getodo;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint16_t anzahl;
    mrc_interpol_seg_t segmente[MRC_INTERPOL_SEG_MAX];
  } __attribute__ ((__packed__)) mr_fahrekurs;

  enum mr_hub_position {
    MR_HUB_OBEN,
    MR_HUB_UNTEN
  };

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    enum mr_hub_position position;
  } __attribute__ ((__packed__)) mr_setze_hubzylinder;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    enum mr_posetyp typ;
    Pose_t pose;
  } __attribute__ ((__packed__)) mr_setze_pose;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_stat_gyro GyroStatus;
    mrc_stat FahrzeugStatus;
    mrc_mode Modus;
  } __attribute__ ((__packed__)) mr_lese_status;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
  } __attribute__ ((__packed__)) mr_header;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_localization_status status;
  } __attribute__ ((__packed__)) mr_localizationstatus_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    Pose_t odometrie;
    Pose_t odogyro;
    PoseV_t odovel;
  } __attribute__ ((__packed__)) mr_lese_alle_posen;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mr_push_t pushmsg;
  } __attribute__ ((__packed__)) mr_push;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    double winkel;
    double geschwindigkeit;
  } __attribute__ ((__packed__)) mr_gyrodata;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    double geschw[4];
  } __attribute__ ((__packed__)) mr_radgeschw;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_laserdatatype_t datatypes;
    mrc_laserdata_t laserdata;
    mrc_laserreflectordata_t reflectordata;
    mrc_lasersegmentdata_t segmentdata;
  } __attribute__ ((__packed__)) mr_laserdata_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_configuredlaser_t configuredlaser;
  } __attribute__ ((__packed__)) mr_configuredlaser_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_status_type_t statustyp;
    int mr_errno;
  } __attribute__ ((__packed__)) mr_status_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_vehicleparameter_t parameter;
  } __attribute__ ((__packed__)) mr_vehicleparameter_paket_t;

#define MR_COURSEFILELIST_MAXFILES 42
  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint8_t num;
    mrc_course_file_t files[MR_COURSEFILELIST_MAXFILES];
  } __attribute__ ((__packed__)) mr_coursefilelist;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    char name[MRC_COURSENAME_MAX];
    mrc_interpol_course_t course;
  } __attribute__ ((__packed__)) mr_getcoursefile;

  typedef enum {
    MR_RESPONSE_SUCCESS,
    MR_RESPONSE_FAILURE,
  } mr_response_code_t;

  #define MR_RESPONSE_MSG_LEN 512

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mr_response_code_t response_code;
    char response_msg[MR_RESPONSE_MSG_LEN];
  } __attribute__ ((__packed__)) mr_response_t;

  typedef enum {
    MR_HELO_NOOPTS    = 0,        /* No options set */
    MR_HELO_REQ_LOGIN = (1<< 1),  /* Peer needs to authenticate */
    MR_HELO_SUP_RTT   = (1<< 2),  /* Peer might send PING packets */
  } mr_helo_opt_t;

#define MR_HELO_NAME_MAX 16

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint16_t proto_vers;
    uint16_t proto_vers_min;
    mr_helo_opt_t options;
    char name[MR_HELO_NAME_MAX];
  } __attribute__ ((__packed__)) mr_helo_t;

  typedef enum {
    MR_AUTH_REQUEST   = (1<< 1),  /* Peer starts authentication */
    MR_AUTH_CHALLENGE = (1<< 2),  /* Peer needs to authenticate */
    MR_AUTH_RESPONSE  = (1<< 3),  /* Peer might send PING packets */
  } mr_auth_type_t;

#define MR_AUTH_AUTHDATA_MAX 32

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mr_auth_type_t type;
    char data[MR_AUTH_AUTHDATA_MAX];
  } __attribute__ ((__packed__)) mr_auth_t;

  typedef struct{
	  enum mr_befehl befehl;
	  struct timeval timestamp;
	  int clientid;
	  uint16_t requestid;
	  mrc_ping_dst dst;
	  struct timeval ts_cli;	/* Timestamp of the client */
	  struct timeval ts_dst;	/* Timestamp of the destination */
  } __attribute__ ((__packed__)) mr_ping_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_motor_current motorcurrent;
  } __attribute__ ((__packed__)) mr_motorcurrent_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_motor_current motorcurrent;
    mrc_motor_actual_pwm_t motorpwm;
    mrc_wheelspeed wheelspeed;
  } __attribute__ ((__packed__)) mr_motordata_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_rfid_scandata_t rfidscan;
  } __attribute__ ((__packed__)) mr_rfidscan_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_rfid_cfg_scanner_t rfidconfig;
  } __attribute__ ((__packed__)) mr_rfidconfig_paket_t;

  typedef struct {
     enum mr_befehl befehl;
     struct timeval timestamp;
     int clientid;
     uint16_t requestid;
     mrc_interpol_mode_t mode;
   } __attribute__ ((__packed__)) mr_interpolmode_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    char filename[MRC_VECTORMAP_FILENAME_MAX];
    mrc_vectormap_t vectormap;
  } __attribute__ ((__packed__)) mr_vectormap_paket_t;

#define MR_VECTORMAPFILELIST_MAXFILES 42

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint8_t nr_files;
    mrc_vectormap_file_t files[MR_VECTORMAPFILELIST_MAXFILES];
  } __attribute__ ((__packed__)) mr_vectormapfilelist;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    double vcc;
    double batt_stat;
  } __attribute__ ((__packed__)) mr_battstat_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_batt_stat_ext_t stat;
  } __attribute__ ((__packed__)) mr_battstat_ext_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
  } __attribute__ ((__packed__)) mr_reset_notauskreis_paket_t;

#define MR_LANDMARKSFILELIST_MAXFILES 42
  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint8_t num;
    mrc_landmarks_file_t files[MR_LANDMARKSFILELIST_MAXFILES];
  } __attribute__ ((__packed__)) mr_landmarksfilelist;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    char name[MRC_LANDMARKPAIR_FILENAME_MAX];
    mrc_landmarks_t landmarks;
  } __attribute__ ((__packed__)) mr_getlandmarksfile;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_rfid_tag_t tag_event;
  } __attribute__ ((__packed__)) mr_rfid_event;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_uwb_scandata_t uwbscan;
  } __attribute__ ((__packed__)) mr_uwbscan_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_uwb_cfg_scanner_t uwbconfig;
  } __attribute__ ((__packed__)) mr_uwbconfig_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_uwb_range_t range;
  } __attribute__ ((__packed__)) mr_uwb_event;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_actuator_list_t list;
  } __attribute__ ((__packed__)) mr_actuator_getlist;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_actuator_status_list_t list;
  } __attribute__ ((__packed__)) mr_actuator_getstatuslist;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_actuator_id_t id;
    mrc_actuator_status_t status;
  } __attribute__ ((__packed__)) mr_actuator_getstatus;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_actuator_id_t id;
    mrc_actuator_position_t dist;
  } __attribute__ ((__packed__)) mr_actuator_getdist;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_actuator_id_t id;
    mrc_actuator_position_t dist;
  } __attribute__ ((__packed__)) mr_actuator_setdist;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint8_t all;
  } __attribute__ ((__packed__)) mr_actuator_reference;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_actuator_id_t id;
    mrc_actuator_config_t config;
  } __attribute__ ((__packed__)) mr_actuator_config;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint32_t segment_nr;
  } __attribute__ ((__packed__)) mr_curinterpolseg_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint8_t nr_progs;
    mrc_svc_prog_t progs[6];
  } __attribute__ ((__packed__)) mr_svc_prog_stat_paket_t;

  enum mr_svc_prog_change {
    MR_SVC_PROG_CHANGE_START,
    MR_SVC_PROG_CHANGE_STOP,
  };

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint8_t progid;
    enum mr_svc_prog_change action;
  } __attribute__ ((__packed__)) mr_svc_prog_change_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_collision_t collision;
  } __attribute__ ((__packed__)) mr_collision_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    uint32_t dst_vertex;
  } __attribute__ ((__packed__)) mr_drivetovertex_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_ublox_hpposecef_t ecef;
  } __attribute__ ((__packed__)) mr_gnss_ecef_event_paket_t;

  typedef struct {
    enum mr_befehl befehl;
    struct timeval timestamp;
    int clientid;
    uint16_t requestid;
    mrc_position_3d_t p0;
    mrc_position_3d_t px;
    mrc_position_3d_t py;
  } __attribute__ ((__packed__)) mr_gnss_corner_update_paket_t;

  typedef union {
    mr_header header;
    mr_setmanuellrks manuellrks;
    mr_setze_modus modus;
    mr_getodo odo;
    mr_fahrekurs kurs;
    mr_setze_hubzylinder setzehub;
    mr_setze_pose pose;
    mr_lese_status status;
    mr_localizationstatus_paket_t localizationstatuspaket;
    mr_lese_alle_posen alleposen;
    mr_push push;
    mr_gyrodata gyrodata;
    mr_radgeschw radgeschw;
    mr_laserdata_paket_t laserpaket;
    mr_configuredlaser_paket_t configuredlaserpaket;
    mr_vehicleparameter_paket_t vehicleparameterpaket;
    mr_status_t statuspaket; 
    mr_coursefilelist coursefilelist;
    mr_getcoursefile getcoursefile;
    mr_response_t response;
    mr_helo_t helo;
    mr_auth_t auth;
    mr_ping_t ping;
    mr_motorcurrent_paket_t motorcurrent;
    mr_rfidscan_paket_t rfidscan;
    mr_rfidconfig_paket_t rfidconfig;
    mr_interpolmode_paket_t interpolmode;
    mr_vectormap_paket_t vectormappaket;
    mr_vectormapfilelist vectormapfilelist;
    mr_motordata_paket_t motordata;
    mr_battstat_paket_t battstat;
    mr_reset_notauskreis_paket_t resetnotaus;
    mr_landmarksfilelist landmarksfilelist;
    mr_getlandmarksfile getlandmarksfile;
    mr_rfid_event rfid_event;
    mr_uwbscan_paket_t uwbscan;
    mr_uwbconfig_paket_t uwbconfig;
    mr_uwb_event uwb_event;
    mr_actuator_getlist actuator_getlist;
    mr_actuator_getstatuslist actuator_getstatuslist;
    mr_actuator_getstatus actuator_getstatus;
    mr_actuator_getdist actuator_getdist;
    mr_actuator_setdist actuator_setdist;
    mr_actuator_reference actuator_reference;
    mr_actuator_config actuator_config;
    mr_battstat_ext_paket_t battstat_ext;
    mr_curinterpolseg_paket_t cursegnr;
    mr_svc_prog_stat_paket_t progstat;
    mr_svc_prog_change_paket_t progaction;
    mr_collision_paket_t collision_event;
    mr_drivetovertex_paket_t drivetovertex;
    mr_gnss_ecef_event_paket_t ecef_event;
    mr_gnss_corner_update_paket_t gnss_corner_update;
  } __attribute__ ((__packed__)) mr_daten_paket;

/*------- Strukturen zum Einlesen und Speichern der Config-Dateien -----*/

/* Key-Sruktur Config-File */
  typedef struct {
    char *key;
    char *value;
    int valcount;
  } key;

/* Gruppen-Struktur Config-File */
  typedef struct {
    char *group;
    key *keys;
    int keycount;
  } fzg_keyvals;

  typedef struct mr_pushpacket {
    struct mr_pushpacket* next;
    mr_daten_paket packet;
  } mr_pushpacket_list_t;

/* Datentypen fuer Debug-Funktionen */
  typedef enum {
    MR_DEBUG_INTTYPE_NORMAL,  /* Normale Fahrt mit dem Interpolator */
    MR_DEBUG_INTTYPE_DIFF     /* Ausgleichsfahrt bei Update der Pose */
  } mr_debug_msg_int_type;

/* Map for active rangings */
  typedef struct {
    uint16_t nr_rangings;
    struct {
      uint64_t src;
      uint64_t dst;
    } rangings[MRC_UWB_CFG_ANCHOR_MAX * MRC_UWB_CFG_SCANNER_MAX];
  } mr_uwb_rangemap;

#if __cplusplus
}
#endif
