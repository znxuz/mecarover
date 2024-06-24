#ifndef MRCLIENT_H
#define MRCLIENT_H

#include <sys/time.h>  /* for struct timespec */
#include <stdint.h>    /* for uint8_t, uint32_t */

#ifndef UINT8_MAX 
  #define UINT8_MAX ((uint8_t)(255U))
#endif

#if __cplusplus
extern "C" {
#endif

/* Library datatype */
  typedef enum {
    MRC_MANUAL,
    MRC_AUTOMATIC,
    MRC_OFF,
    MRC_NEUTRAL,
    MRC_HOLD,
    MRC_ESTOP
  } mrc_mode;

  typedef enum {
    MRC_GYRO_NOERR,
    MRC_GYRO_NOINIT,
    MRC_GYRO_COMM_CRC,
    MRC_GYRO_COMM_TIMEOUT,
    MRC_GYRO_BIGVAL
  } mrc_stat_gyro;

  typedef enum {
    MRC_NOERR,
    MRC_DRZERR,
    MRC_LAGEERR,
    MRC_COMMERR,
    MRC_DOCKERR,
    MRC_EMGOFF,
    MRC_NOPERM
  } mrc_stat;

  typedef struct {
    mrc_mode mode;
    mrc_stat_gyro gyro;
    mrc_stat stat;
  } mrc_stat_all_t;

  typedef enum {
    MRC_LOCALIZATION_OFF,
    MRC_LOCALIZATION_NO_CHANGE,
    MRC_LOCALIZATION_GLOBAL_LASER,
    MRC_LOCALIZATION_LOCAL,
    MRC_LOCALIZATION_LOCAL_UPDATE
  } mrc_localization_modes;

#if !defined(SWIGJAVASCRIPT)
  typedef enum {
    MRC_CBACK_ODO,
    MRC_CBACK_ODO_VEL,
    MRC_CBACK_ODO_GYRO,
    MRC_CBACK_ALLPOSE,
    MRC_CBACK_STATUS,
    MRC_CBACK_LOCALIZATIONSTATUS,
    MRC_CBACK_GYRO,
    MRC_CBACK_WHELLSPEED,
    MRC_CBACK_LASERALLDATA,
    MRC_CBACK_STATUS_UPDATE,
    MRC_CBACK_RKS_VEL,
    MRC_CBACK_MANUALRKS_SET,
    MRC_CBACK_MOTORCURRENT,
    MRC_CBACK_BATTSTAT,
    MRC_CBACK_RFID_EVENT,
    MRC_CBACK_UWB_EVENT,
    MRC_CBACK_ACTUATOR_STATUSLIST,
    MRC_CBACK_BATTSTAT_EXT,
    MRC_CBACK_COLLISION_EVENT,
    MRC_CBACK_GNSS_ECEF_EVENT,
    MRC_CBACK_MODE_EVENT,
/* Add new elements bevore this line */
    MRC_CBACK_LASTOP
  } mrc_callback_type;
#endif

  typedef enum {
	  MRC_PING_SRV,
	  MRC_PING_CLC,
    MRC_PING_CLIENT,
  } mrc_ping_dst;

  /* mrc_client_connection_t Datentypen */
  typedef void* mrc_client_connection_t;

#if !defined(SWIGJAVASCRIPT)
  typedef int (*mrc_callback_func) (mrc_client_connection_t con, void *data, mrc_callback_type type,
				    struct timeval timestamp);
  typedef int (*mrc_callback_func_ud) (mrc_client_connection_t con, void *data, mrc_callback_type type, struct timeval timestamp, void *userdata);
#endif

  typedef struct {
    double x;
    double y;
    double theta;
  } mrc_pose;

  typedef struct {
    double x;
    double y;
    double z;
  } mrc_position_3d_t;

  typedef struct {
    double vx;
    double vy;
    double omega;
  } mrc_vel;

  typedef struct {
    double v;
    double omega;
  } mrc_pathvel;

  typedef struct {
    mrc_pose pose;
    mrc_vel vel;
  } mrc_pose_vel;

  typedef struct {
    mrc_pose odo;
    mrc_pose gyro;
    mrc_pose_vel odovel;
  } mr_odo_all_t;

  typedef struct {
    mrc_pose pose;
    mrc_pathvel vel;
  } mrc_pose_pathvel;

  typedef enum {
    MRC_LOADLIFT_UP,
    MRC_LOADLIFT_DOWN
  } mrc_loadliftpos;

/* --- actuators --- */
  #define MRC_ACTUATOR_ID_MAX 10
  #define MRC_ACTUATOR_NAME_MAX 25

  typedef uint8_t mrc_actuator_id_t;

  typedef enum {
    MRC_ACTUATOR_OFF = 0,
    MRC_ACTUATOR_HOMING,
    MRC_ACTUATOR_INITIALIZED,
    MRC_ACTUATOR_RUNNING,
    MRC_ACTUATOR_ERROR,
  } mrc_actuator_status_t;

  typedef char mrc_actuator_name_t[MRC_ACTUATOR_NAME_MAX];

  typedef float mrc_actuator_position_t;

  typedef struct {
    mrc_actuator_id_t id;
    mrc_actuator_name_t name;
  } mrc_actuator_id_name_t;

  typedef struct {
    mrc_actuator_id_t id;
    mrc_actuator_status_t status;
    mrc_actuator_position_t position;
  } mrc_actuator_id_status_position_t;

  typedef struct {
    mrc_actuator_id_t id;
    mrc_actuator_position_t position;
    uint8_t wait;
  } mrc_actuator_id_position_t;

  typedef struct {
    uint8_t noactuators;
    mrc_actuator_id_name_t actuator[MRC_ACTUATOR_ID_MAX];
  } mrc_actuator_list_t;

  typedef struct {
    uint8_t noactuators;
    mrc_actuator_id_status_position_t actuator[MRC_ACTUATOR_ID_MAX];
  } mrc_actuator_status_list_t;

  typedef struct {
    uint32_t rawposition;
    int32_t HomingOffset;
    double factor;
    double groundcontact;
  } mrc_actuator_config_t;

  typedef int32_t mrc_actuator_current_t[MRC_ACTUATOR_ID_MAX];
  typedef float mrc_actuator_temperature_t[MRC_ACTUATOR_ID_MAX];

/* --- END --- */

  typedef struct {
    struct timeval timestamp;
    double angle;
    double speed;
  } mrc_gyrodata;

  typedef struct {
    struct timeval timestamp;
    double speeds[4];
  } mrc_wheelspeed;

  typedef struct {
    mrc_localization_modes mode;
    mrc_pose startpose;
    struct timeval timestamp;
  } mrc_localization_status;

  /* Status and errno types */
  typedef enum {
    MRC_REGLER_ERROR,
    MRC_GYRO_ERROR,
    MRC_INTERPOL_ERROR,
    MRC_INTERPOL_STATUS
  } mrc_status_type_t;

  typedef struct {
    mrc_status_type_t statustyp;
    int mr_errno;
  } mrc_status;

/* --- Interpolator --- */
  #define MRC_INTERPOL_SEG_MAX_POINTS 10
  #define MRC_INTERPOL_SEG_MAX 1024

  /* mrc_interpol_state_t */
  #define MRC_INT_CRS_FINISHED 1
  #define MRC_INT_CRS_ABORTED 2
  #define MRC_INT_CRS_BREAKDOWN 3
  #define MRC_INT_CRS_RESTART 4
  #define MRC_INT_CRS_SEGFINISHED 5

  /* mrc_interpol_err_t */
  #define MRC_INT_ERR_WAY_SPEED_DIS 0
  #define MRC_INT_ERR_NO_ACCEL_PHASE 1
  #define MRC_INT_ERR_RADIUS_SIZE 2
  #define MRC_INT_ERR_RELATION_ERR 3
  #define MRC_INT_ERR_GET_LOOKUP 4
  #define MRC_INT_ERR_READ_ODO 5
  #define MRC_INT_ERR_SET_SETPOINT 6
  #define MRC_INT_ERR_CLC_AUTO 7
  #define MRC_INT_ERR_CLC_OFF 8
  #define MRC_INT_ERR_PROT_FIELD 9
  #define MRC_INT_ERR_LIFT_CYLINDER 10
  #define MRC_INT_ERR_CLC_HOLD 11
  #define MRC_INT_ERR_FINE_INT_V2 12
  #define MRC_INT_ERR_SPLINE_DIST_ERR 13
  #define MRC_INT_ERR_SPEED_VEL_TOO_HIGH 14

  typedef enum {
    MRC_INTERPOL_MODE_ABORT,
    MRC_INTERPOL_MODE_PAUSE,
    MRC_INTERPOL_MODE_CONTINUE,
  } mrc_interpol_mode_t;

  typedef enum {
    MRC_INTERPOL_SEG_LINE,
    MRC_INTERPOL_SEG_LOADLOAD,
    MRC_INTERPOL_SEG_LOADUNLOAD,
    MRC_INTERPOL_SEG_CIRCLE,
    MRC_INTERPOL_SEG_START,
    MRC_INTERPOL_SEG_SPLINE,
    MRC_INTERPOL_SEG_CROSSING,
    MRC_INTERPOL_SEG_QUINTICSPLINE,
  } mrc_interpol_seg_type_t;

  typedef enum {
    MRC_INTERPOL_INTERPOLTYPE_LINEAR,
    MRC_INTERPOL_INTERPOLTYPE_SINUS,
    MRC_INTERPOL_INTERPOLTYPE_VSLINEAR,
  } mrc_interpol_interpolation_type_t;
  
  typedef enum {
    MRC_INTERPOL_FEININTERPOLTYPE_UPDATE,
    MRC_INTERPOL_FEININTERPOLTYPE_PERIODE,
  } mrc_interpol_feininterpol_type_t;

  typedef enum {
    MRC_INTERPOL_SEG_PARAM_FIELDSIZE_LARGE = 0,
    MRC_INTERPOL_SEG_PARAM_FIELDSIZE_SMALL = 1,
  } mrc_interpol_seg_param_fieldsize_t;

  typedef enum {
    MRC_INTERPOL_SEG_PARAM_WARNFIELD_DEACTIVED = 0,
    MRC_INTERPOL_SEG_PARAM_WARNFIELD_BREAKDOWN = 1,
  } mrc_interpol_seg_param_warnfield_t;

  typedef struct {
    double v_max;
    double a;
    double alpha_max;
    double omega_max;
    double r;
    uint8_t n; // Anzahl der Stuetzpunkte bei einem Spline-Segment
    uint32_t wait;
    mrc_interpol_interpolation_type_t interpol;  // Typ des zu verwendenden Interpolators    
    mrc_interpol_feininterpol_type_t feininterpol;
    mrc_localization_modes loctype;
    mrc_interpol_seg_param_fieldsize_t fieldsize;
    mrc_interpol_seg_param_warnfield_t warnfieldtype;
    uint8_t noactuators;
  } mrc_interpol_seg_param_t;

  typedef struct {
    mrc_interpol_seg_type_t type;
    mrc_pose_pathvel start;
    mrc_pose_pathvel points[MRC_INTERPOL_SEG_MAX_POINTS];
    mrc_pose_pathvel end;
    mrc_interpol_seg_param_t param;
    mrc_actuator_id_position_t actuator[MRC_ACTUATOR_ID_MAX];
  } mrc_interpol_seg_t;

  typedef struct {
    uint16_t count;
    mrc_interpol_seg_t segments[MRC_INTERPOL_SEG_MAX];
  } mrc_interpol_course_t;

#if !defined(SWIG)
  typedef struct {
    uint8_t interpolnr;
    uint16_t segmentnr;
    double total_dist;
    double total_time;
    double curr_dist;
    double curr_time;
    mrc_pose pose;
    enum {
      MRC_INTERPOL_STATE_IDLE,
      MRC_INTERPOL_STATE_RUNNING,
      MRC_INTERPOL_STATE_SLOWDOWN,
      MRC_INTERPOL_STATE_STOPED
    } state;
  } mrc_interpol_status_t;
#endif

/* --- End: Interpolator --- */

#define MRC_SCANNER_MAX 6
#define MRC_BEAMS_MAX 2048
#define MRC_LASERSEGMENTS_MAX 30
#define MRC_LASERREFLECTORS_MAX 30
	
  /* Enumeration zur Beschreibung besonderer Eigenschaften eines einzelnen Laserstrahls */
  #define MRC_ON_REFLECTOR_TAPE (1 << 1)
  #define MRC_IN_WARNING_FIELD (1 << 2)
  #define MRC_IN_SECURITY_FIELD (1 << 3)
  #define MRC_LASERBEAM_ATTRIBUTE_DIST_INF (1 << 4)		/* Beam did not strike anything at all */
  #define MRC_LASERBEAM_ATTRIBUTE_DIST_INF_SIGMA_1 (1 << 5)	/* Beam did not strike anything at all */
  #define MRC_LASERBEAM_ATTRIBUTE_DIST_INF_SIGMA_2 (1 << 6)	/* Beam did not strike anything at all */
  #define MRC_LASERBEAM_ATTRIBUTE_DIST_INF_SIGMA_3 (1 << 7)	/* Beam did not strike anything at all */
  #define MRC_LASERBEAM_ATTRIBUTE_DIST_INF_SIGMA_4 (1 << 8)	/* Beam did not strike anything at all */
  #define MRC_LASERBEAM_ATTRIBUTE_DIST_INF_SIGMA_5 (1 << 9)	/* Beam did not strike anything at all */
  #define MRC_LASERBEAM_ATTRIBUTE_ON_COMMON_AREA (1 << 10)      /* Beam did striked in an overlapping area */
  typedef unsigned int mrc_laserbeam_attribute_t;

  /* Type-Definition fuer den Filter-Typ */
  typedef enum{
    MRC_LASERFILTER_NONE   = (1 << 0),
    MRC_LASERFILTER_MEDIAN = (1 << 1),
    MRC_LASERFILTER_GAUSS  = (1 << 2)
  } mrc_laserfilter_t;

  /* Type-Definition fuer den Daten-Modus des Laserscanners */
  typedef enum{
    MRC_LASERDATAMODE_ALL_DATA       = (1 << 0),
    MRC_LASERDATAMODE_REFLECTOR_DATA = (1 << 1)
  } mrc_laserdatamode_t;

  /* Type-Definition fuer den Typ  */
  #define MRC_LASERDATATYPE_REFLECTORS  (1 << 1)
  #define MRC_LASERDATATYPE_SEGMENTS    (1 << 2)
  #define MRC_LASERDATATYPE_BEAMS       (1 << 3)
  typedef unsigned int mrc_laserdatatype_t;

  /* Type-Definition ob die Daten in den Mittelpunkkt des Roboters transformiert wurden */
  typedef enum{
    MRC_LASERTRANSFORM_ON  = (1 << 0),
    MRC_LASERTRANSFORM_OFF = (1 << 1)
  } mrc_lasertransform_t;

#define MRC_LASERCONFIG_PORT_MAX 128

  typedef enum {
    MRC_SERIAL_INTERFACE_USB,
    MRC_SERIAL_INTERFACE_SIM,
    MRC_SERIAL_INTERFACE_RS422,
    MRC_SERIAL_INTERFACE_TCP,
  } mrc_serial_interface_t;

  /* Typ zur Beschreibung eines Laserscanners */
  typedef struct{
    mrc_laserfilter_t filter;
    mrc_laserdatamode_t mode;
    mrc_lasertransform_t transform;
    int xoffset;
    int yoffset;
    int maxdistance;
    float resolution;
    double sigma_d;
    double sigma_alpha;
    double phioffset;
    char type[10];
    char port[MRC_LASERCONFIG_PORT_MAX];
    mrc_serial_interface_t iface;
    uint32_t baud;
    uint8_t syserr;
    uint8_t sigma[5];
  } mrc_laserconfig_t;

  typedef struct{
    struct timeval timestamp;
    short noscanners;
    mrc_laserconfig_t scanner[MRC_SCANNER_MAX];
  } mrc_configuredlaser_t;

  /* Typ zur Beschreibung eines einzelnen Laserstrahls */
  typedef struct {
    int distance;
    int x;
    int y;
    double angle;
    mrc_laserbeam_attribute_t attribute;
  } mrc_laserbeam_t;

  /* Typ zur Beschreibung der Messung eines einzelnen Laserscanners.
   * Die darin angegebenen Offset-Werte werden in der Konfigurationsdatei des
   * Fahrzeugs angegeben. Des Weiteren enthaelt dieser Typ die Odometrie-Pose
   * und die korrigierte Odometrie-Pose fuer die Positionsbestimmung.
  */
  typedef struct {
    struct timeval timestamp;
    mrc_pose odo_pose;
    mrc_pose_vel current_pose;
    short nobeams;
    mrc_laserbeam_t beams[MRC_BEAMS_MAX];
  } mrc_laserscan_t;

  /* Typ zur Beschreibung der Messungen von bis zu MRC_SCANNER_MAC Messungen */
  typedef struct {
    struct timeval timestamp;
    short noscans;
    mrc_laserscan_t scan[MRC_SCANNER_MAX];
  } mrc_laserdata_t;

  /* Typ zur Beschreibung der Messung eines einzelnen Laserscanners.
   * Die darin angegebenen Offset-Werte werden in der Konfigurationsdatei des
   * Fahrzeugs angegeben. Des Weiteren enthaelt dieser Typ die Odometrie-Pose
   * und die korrigierte Odometrie-Pose fuer die Positionsbestimmung.
  */
  typedef struct {
    struct timeval timestamp;
    mrc_pose odo_pose;
    mrc_pose_vel current_pose;
    uint16_t noreflectors;
    mrc_laserbeam_t centers[MRC_LASERREFLECTORS_MAX];
  } mrc_laserreflectorscan_t;

  /* Typ zur Beschreibung der Messungen von bis zu MRC_SCANNER_MAC Messungen */
  typedef struct {
    struct timeval timestamp;
    uint16_t noscans;
    mrc_laserreflectorscan_t scan[MRC_SCANNER_MAX];
  } mrc_laserreflectordata_t;

  /* Typ zur Beschreibung eines Laser-Segments in Hesse-Normalform */
  typedef struct {
    short start_index;
    short end_index;
    int xstart;
    int ystart;
    int xend;
    int yend;
    int length;
    int distance;
    double angle;
    double sigma_p;
    double sigma_alpha;
    double sigma_palpha;
  } mrc_lasersegment_t;

  /* Typ zur Beschreibung aller Lasersegmente eines Laserscanners.
   * Dabei wird auch in diesem Datentyp die Odometrie- sowie die korrigierte Pose
   * mitgeliefert um diese fuer die Lokalisierung zu verwenden.
  */
  typedef struct{
    struct timeval timestamp;
    mrc_pose odo_pose;
    mrc_pose_vel current_pose;
    uint16_t nosegments;
    mrc_lasersegment_t segments[MRC_LASERSEGMENTS_MAX];
   } mrc_lasersegmentscan_t;

  /* Typ zur Beschreibung der erkannten Segemente von bis zu MRC_SCANNER_MAC Laserscannern */
  typedef struct {
    struct timeval timestamp;
    uint16_t noscans;
    mrc_lasersegmentscan_t scan[MRC_SCANNER_MAX];
  } mrc_lasersegmentdata_t;

  /* Type for laserdata callback */
  typedef struct {
    mrc_laserdata_t laserdata;
    mrc_laserreflectordata_t laserreflectordata;
    mrc_lasersegmentdata_t lasersegmentdata;
  } mrc_laseralldata_t;

  #define MRC_LANDMARKNAME_MAX 42

  typedef struct {
    int xleft;
    int yleft;
    int xright;
    int yright;
    int search_area;
    double orientation;
    uint8_t district_number;
    char name[MRC_LANDMARKNAME_MAX];
  } mrc_landmarkpair_t;

  #define MRC_LANDMARKS_MAX UINT8_MAX

  typedef struct {
    uint8_t nopairs;
    mrc_landmarkpair_t pairs[MRC_LANDMARKS_MAX];
  } mrc_landmarks_t;

  /* Typedefinition for courses files */
  #define MRC_LANDMARKPAIR_FILENAME_MAX 512

  typedef enum {
    MRC_LANDMARKS_FILE_TYPE_FILE,
    MRC_LANDMARKS_FILE_TYPE_FOLDER,
    MRC_LANDMARKS_FILE_TYPE_UNKNOWN,
  } mrc_landmarks_file_type_t;

  typedef struct {
    char name[MRC_LANDMARKPAIR_FILENAME_MAX];
    mrc_landmarks_file_type_t type;
    uint8_t pairs;
  } mrc_landmarks_file_t;

  #define MRC_UWBANCHORMAP_NAME_MAX 42
  #define MRC_UWB_CFG_ANCHOR_MAX 50

  typedef struct {
    char name[MRC_UWBANCHORMAP_NAME_MAX];
    uint64_t id;
    struct {
      int x;
      int y;
      int z;
    } position;
    double orientation;
    uint8_t nodistanceoffsets;
    struct {
      uint64_t id;
      double offset;
    } distanceoffset[MRC_UWB_CFG_ANCHOR_MAX * 10];
  } mrc_uwbanchor_t;

  #define MRC_UWBANCHORMAP_ANCHORS_MAX UINT8_MAX

  typedef struct {
    uint8_t noanchors;
    mrc_uwbanchor_t anchors[MRC_UWBANCHORMAP_ANCHORS_MAX];
  } mrc_uwbanchormap_t;

  /* Defines a type that can store on RFID tag's UID */
  typedef uint64_t mrc_rfid_uid_t;

  /* Defines a tag type enumeration */
  typedef enum {
    MRC_RFID_TYPE_UNKNOWN = 0,
    MRC_RFID_TYPE_ISO15693 = 1,
    MRC_RFID_TYPE_ISO14443 = 2,
    MRC_RFID_TYPE_MIFARE_1K = 3,
    MRC_RFID_TYPE_MIFARE_4K = 4,
    MRC_RFID_TYPE_MIFARE_PLUS = 5,
    MRC_RFID_TYPE_MIFARE_DESFIRE = 6
  } mrc_rfid_type_t;

  /* Type for storing RSSI values */
  typedef uint8_t mrc_rfid_rssi_t[2];

  typedef enum {
    MRC_RFID_TAG_EVENT_ADD = 0,
    MRC_RFID_TAG_EVENT_REM = 1,
  } mrc_rfid_tag_event_t;

  /* Defines a data type for a single detected RFID tag */
  typedef struct {
    struct timeval timestamp;
    uint8_t scannerid;
    uint8_t antennaid;
    mrc_pose odo_pose;
    mrc_pose_vel current_pose;
    mrc_rfid_type_t type;
    mrc_rfid_uid_t uid;
    mrc_rfid_rssi_t rssi;
    mrc_rfid_tag_event_t event;
  } mrc_rfid_tag_t;

#define MRC_RFID_SCAN_UID_MAX 8

  /* Defines a single RFID scan */
  typedef struct {
    uint8_t notags;
    mrc_rfid_tag_t tag[MRC_RFID_SCAN_UID_MAX];
  } mrc_rfid_scan_t;

#define MRC_RFID_CFG_PORT_MAX 128
#define MRC_RFID_CFG_TYPE_MAX 42
#define MRC_RFID_CFG_SCANNER_MAX 5

  typedef enum {
    MRC_RFID_CFG_MODE_TAG_ONE,
    MRC_RFID_CFG_MODE_TAG_ALL,
    MRC_RFID_CFG_MODE_TAG_ALL_CONT,
  } mrc_rfid_cfg_mode_t;

  /* Typ zur Beschreibung eines Laserscanners */
  typedef struct {
    int16_t xoffset;
    int16_t yoffset;
    char type[MRC_RFID_CFG_TYPE_MAX];
    char port[MRC_RFID_CFG_PORT_MAX];
    mrc_serial_interface_t iface;
    uint32_t baud;
    mrc_rfid_cfg_mode_t mode;
    uint8_t tag_afterglow;
  } mrc_rfid_cfg_t;

  typedef struct {
    uint8_t noscanners;
    mrc_rfid_cfg_t scanner[MRC_RFID_CFG_SCANNER_MAX];
  } mrc_rfid_cfg_scanner_t;

  typedef struct {
    uint8_t noscans;
    mrc_rfid_scan_t scan[MRC_RFID_CFG_SCANNER_MAX];
  } mrc_rfid_scandata_t;


/* GNSS Navigon */
#define MRC_GNSS_CFG_PORT_MAX 128
#define MRC_GNSS_CFG_TYPE_MAX 42

  typedef struct {
    int16_t xoffset;
    int16_t yoffset;
    char type[MRC_GNSS_CFG_TYPE_MAX];
    char port[MRC_GNSS_CFG_PORT_MAX];
    mrc_serial_interface_t iface;
    uint32_t baud;
  } mrc_gnss_cfg_t;

  typedef struct {
    struct timeval timestamp;
    mrc_pose odo_pose;
    mrc_pose_vel current_pose;
    double latitude;
    double longitude;
    uint8_t quality;
    uint8_t nr_sats;
    double hdop;
    double height;
    struct timespec gpstime;
  } mrc_gnss_scan_t;

  typedef struct {
    uint8_t type;
    uint8_t valid;
    uint8_t dgnss_active;
  } mrc_ublox_fixstatus_t;

  typedef struct {
    struct timeval timestamp;
    double x_ecef;  // ECEF + ECEFHp in mm
    double y_ecef;  // ECEF + ECEFHp in mm
    double z_ecef;  // ECEF + ECEFHp in mm
    uint32_t pos_acc;  // in mm
    mrc_ublox_fixstatus_t status;
    struct timespec gpstime;
  } mrc_ublox_hpposecef_t;



/* UWB */
#define MRC_UWB_CFG_PORT_MAX 128
#define MRC_UWB_CFG_TYPE_MAX 42
#define MRC_UWB_CFG_SCANNER_MAX 6

  /* Typ zur Beschreibung eines UWB-Moduls */
  typedef struct {
    int16_t xoffset;
    int16_t yoffset;
    int16_t zoffset;
    double thetaOffset;
    char type[MRC_UWB_CFG_TYPE_MAX];
    char port[MRC_UWB_CFG_PORT_MAX];
    mrc_serial_interface_t iface;
    uint32_t baud;
    uint64_t id;
  } mrc_uwb_cfg_t;

  typedef struct {
    uint8_t noscanners;
    mrc_uwb_cfg_t scanner[MRC_UWB_CFG_SCANNER_MAX];
  } mrc_uwb_cfg_scanner_t;

  typedef struct {
    struct timeval timestamp;
    mrc_pose odo_pose;
    mrc_pose_vel current_pose;
    uint8_t scannerid;
    uint64_t src;
    uint64_t dst;
    uint8_t status;
    uint32_t dist;
    int16_t rssi;
  } mrc_uwb_range_t;

  typedef struct {
    uint8_t noranges;
    mrc_uwb_range_t range[MRC_UWB_CFG_ANCHOR_MAX];
  } mrc_uwb_scan_t;

  typedef struct {
    uint8_t noscans;
    mrc_uwb_scan_t scan[MRC_UWB_CFG_SCANNER_MAX];
  } mrc_uwb_scandata_t;

#define MRC_VECTORMAP_LINES_MAX 500
#define MRC_VECTORMAP_GRIDS_MAX 20
#define MRC_VECTORMAP_FILENAME_MAX 50

  /* */
  typedef struct {
    uint16_t no_xgrids;
    uint16_t no_ygrids;
    int32_t xmin;
    int32_t ymin;
    int32_t xmax;
    int32_t ymax;
    double xgrid_size;
    double ygrid_size;
  } mrc_vectormap_properties_t;

  /* */
  typedef struct {
    uint32_t length;
    uint32_t distance;
    uint8_t no_grids;
    int16_t grids[MRC_VECTORMAP_GRIDS_MAX];
    int32_t x_start;
    int32_t y_start;
    int32_t x_end;
    int32_t y_end;
    double angle;
    double norm_x;
    double norm_y;
  } mrc_vectormap_line_t;

  /* */
  typedef struct {
    mrc_vectormap_properties_t mapproperties;
    uint32_t no_lines;
    mrc_vectormap_line_t lines[MRC_VECTORMAP_LINES_MAX];
  } mrc_vectormap_t;

  typedef enum {
    MRC_VECTORMAP_FILE_TYPE_FILE,
    MRC_VECTORMAP_FILE_TYPE_FOLDER,
    MRC_VECTORMAP_FILE_TYPE_UNKNOWN,
  } mrc_vectormap_file_type_t;

  typedef struct {
    char name[MRC_VECTORMAP_FILENAME_MAX];
    mrc_vectormap_file_type_t type;
  } mrc_vectormap_file_t;

  /* Typedefinition for courses files */
  #define MRC_COURSENAME_MAX 512

  typedef enum {
    MRC_COURSES_FILE_TYPE_FILE,
    MRC_COURSES_FILE_TYPE_FOLDER,
    MRC_COURSES_FILE_TYPE_UNKNOWN,
  } mrc_course_file_type_t;

  typedef struct {
    char name[MRC_COURSENAME_MAX];
    mrc_course_file_type_t type;
    uint8_t points;
    uint32_t length;
  } mrc_course_file_t;

  typedef enum {
    MRC_VEHICLETYPE_UNKNOWN = 0,
    MRC_VEHICLETYPE_MECANUM = 1,
    MRC_VEHICLETYPE_DIFFERENTIAL = 2,
  } mrc_vehicletype_t;

  typedef struct {
    mrc_vehicletype_t type;
    double length;
    double width;
    double wheelradius;
    double rollradius;
    double max_v;
    double max_omega;
    double ratio;
    uint16_t inc;
  } mrc_vehicleparameter_t;

  typedef enum {
    MRC_CON_FAILURE   = -1,         /* Server requires login */
    MRC_CON_SUCCESS   = 0,          /* Server requires login */
    MRC_CON_LOGIN_REQ = (1 << 0),   /* Server requires login */
  } mrc_connect_t;

  typedef int32_t mrc_motor_current[4];
  typedef int16_t mrc_motor_actual_pwm_t[4];
  typedef float mrc_motor_temperature_t[4];
  typedef float mrc_motorresistor_temperature_t[4];

  typedef struct {
    double vcc;
    double batt_stat;
  } mrc_batt_stat_t;

  typedef struct {
    uint8_t chargelevel;   // in % [0 100]
    double vcc;            // in volts [0 ~30]
    double capacity;       // remaning capacity in Ah
    double discharge_rate; // Current discharge current in A
    uint32_t time_remain;  // Seconds till fully discharged (guess)
  } mrc_batt_stat_ext_t;

/* ------ Radar ------ */

/* ------ SVC ------ */
#define MRC_SVC_MAXNAME 32

typedef enum {
  MRC_SVC_NOT_INIT = 0,
  MRC_SVC_STARTED,
  MRC_SVC_DIED,
  MRC_SVC_EXITED,
} mrc_svc_stat_t;

typedef struct {
  char name[MRC_SVC_MAXNAME];
  mrc_svc_stat_t stat;
  uint8_t id;
} mrc_svc_prog_t;

/* ------ SVC ------ */

typedef enum {
  MRC_RADAR_DYNPROP_MOVING = 0x0,
  MRC_RADAR_DYNPROP_STATIONARY = 0x1,
  MRC_RADAR_DYNPROP_ONCOMING = 0x2,
  MRC_RADAR_DYNPROP_STATIONARY_CANDIDATE = 0x3,
  MRC_RADAR_DYNPROP_UNKNOWN = 0x4,
  MRC_RADAR_DYNPROP_CROSSING_STATIONARY = 0x5,
  MRC_RADAR_DYNPROP_CROSSING_MOVING = 0x6,
  MRC_RADAR_DYNPROP_STOPPED = 0x7
} mrc_radar_dynprop_t;

typedef enum {
  MRC_RADAR_MEASSTATE_DELETED = 0x0,
  MRC_RADAR_MEASSTATE_NEW = 0x1,
  MRC_RADAR_MEASSTATE_MEASURED = 0x2,
  MRC_RADAR_MEASSTATE_PREDICTED = 0x3,
  MRC_RADAR_MEASSTATE_DELETED_FOR_MERGE = 0x4,
  MRC_RADAR_MEASSTATE_NEW_FROM_MERGE = 0x5
} mrc_radar_measstate_t;

typedef enum {
  MRC_RADAR_CLASS_POINT = 0x0,
  MRC_RADAR_CLASS_CAR = 0x1,
  MRC_RADAR_CLASS_TRUCK = 0x2,
  MRC_RADAR_CLASS_PEDESTRIAN = 0x3,
  MRC_RADAR_CLASS_MOTORCYCLE = 0x4,
  MRC_RADAR_CLASS_BICYCLE = 0x5,
  MRC_RADAR_CLASS_WIDE = 0x6,
  MRC_RADAR_CLASS_RESERVED = 0x7
} mrc_radar_class_t;

typedef enum {
  MRC_RADAR_INVALID_STATE_VALID = 0x00,
  MRC_RADAR_INVALID_STATE_INVALID_LOW_RCS = 0x01,
  MRC_RADAR_INVALID_STATE_INVALID_NEARFIELD_ARTEFACT = 0x02,
  MRC_RADAR_INVALID_STATE_INVALID_NOT_CONFIRMED_NEARRANGE = 0x03,
  MRC_RADAR_INVALID_STATE_VALID_LOW_RCS = 0x04,
  MRC_RADAR_INVALID_STATE_RESERVED_1 = 0x05,
  MRC_RADAR_INVALID_STATE_INVALID_HIGH_MIRROR_PROBABILITY = 0x06,
  MRC_RADAR_INVALID_STATE_INVALID_OUTSIDE_FIELDOFVIEW = 0x07,
  MRC_RADAR_INVALID_STATE_VALID_AZIMUTH_CORRECTION = 0x08,
  MRC_RADAR_INVALID_STATE_VALID_HIGH_CHILD_PROBABILITY = 0x09,
  MRC_RADAR_INVALID_STATE_VALID_HIGH_PROBABILITY_50_DEG_ARTEFACT = 0x0A,
  MRC_RADAR_INVALID_STATE_VALID_NO_LOCAL_MAXIMUM = 0x0B,
  MRC_RADAR_INVALID_STATE_VALID_HIGH_ARTEFACT_PROBABILITY = 0x0C,
  MRC_RADAR_INVALID_STATE_RESERVED_2 = 0x0D,
  MRC_RADAR_INVALID_STATE_INVALID_HARMONICS = 0x0E,
  MRC_RADAR_INVALID_STATE_VALID_ABOVE_95_M = 0x0F,
  MRC_RADAR_INVALID_STATE_VALID_HIGH_MULTITARGET_PROBABILITY = 0x10,
  MRC_RADAR_INVALID_STATE_VALID_SUSPICIOUS_ANGLE = 0x11
} mrc_radar_invalid_state_t;

typedef enum {
  MRC_RADAR_PROBABILITY_INVALID = 0x0,
  MRC_RADAR_PROBABILITY_25 = 0x1,
  MRC_RADAR_PROBABILITY_50 = 0x2,
  MRC_RADAR_PROBABILITY_75 = 0x3,
  MRC_RADAR_PROBABILITY_90 = 0x4,
  MRC_RADAR_PROBABILITY_99 = 0x5,
  MRC_RADAR_PROBABILITY_99_9 = 0x6,
  MRC_RADAR_PROBABILITY_100 = 0x7
} mrc_radar_probability_t;

typedef enum {
  MRC_RADAR_AMBIG_STATE_INVALID = 0x0,
  MRC_RADAR_AMBIG_STATE_AMBIGUOUS = 0x1,
  MRC_RADAR_AMBIG_STATE_STAGGERED_RAMP = 0x2,
  MRC_RADAR_AMBIG_STATE_UNAMBIGUOUS = 0x3,
  MRC_RADAR_AMBIG_STATE_STATIONARY_CANDIDATES = 0x4,
} mrc_ambig_state_t;

typedef struct {
  int32_t dist_long;
  int32_t dist_lat;
  int32_t vrel_long;
  int32_t vrel_lat;
  uint16_t dist_long_rms;
  uint16_t dist_lat_rms;
  uint16_t vrel_long_rms;
  uint16_t vrel_lat_rms;
  float rcs;
  mrc_radar_dynprop_t dynprop;
  mrc_radar_probability_t pdh0;
  mrc_ambig_state_t ambigstate;
  mrc_radar_invalid_state_t invalidstate;
} mrc_radar_cluster_t;

typedef struct {
  uint8_t id;
  int32_t dist_long;
  int32_t dist_lat;
  int32_t vrel_long;
  int32_t vrel_lat;
  uint16_t dist_long_rms;
  uint16_t dist_lat_rms;
  uint16_t vrel_long_rms;
  uint16_t vrel_lat_rms;
  uint16_t arel_long_rms;
  uint16_t arel_lat_rms;
  float rcs;
  double orientationangle;
  double orientation_rms;
  int16_t arel_long;
  int16_t arel_lat;
  uint16_t length;
  uint16_t width;
  mrc_radar_dynprop_t dynprop;
  mrc_radar_measstate_t measstate;
  mrc_radar_probability_t probofexist;
  mrc_radar_class_t object_class;
} mrc_radar_object_t;

/* -- Collision detection -- */

typedef struct {
  double angle; /* Angle from where the collision hit */
  double amount; /* The amount of collision in mm (the amount of cover displacement) */
} mrc_collision_t;

/* Library functions */
  extern mrc_client_connection_t mrc_client_connection_init(void);
  extern mrc_connect_t mrc_connect(mrc_client_connection_t con, const char *cliname);
  extern mrc_connect_t mrc_connect_tcp(mrc_client_connection_t con, const char *host, const char *port, const char *cliname);
  extern mrc_connect_t mrc_connect_fd(mrc_client_connection_t con, const char *cliname, int fd);
  extern int mrc_geteventfd(mrc_client_connection_t con);
  extern int mrc_disconnect(mrc_client_connection_t con);
  extern int mrc_setmode(mrc_client_connection_t con, mrc_mode mode);
  extern int mrc_setmanualrks(mrc_client_connection_t con, mrc_vel velocity);
  extern int mrc_setloadlift(mrc_client_connection_t con, mrc_loadliftpos pos);
  extern int mrc_getodo(mrc_client_connection_t con, mrc_pose * pose, struct timeval *tv);
  extern int mrc_getodo_gyro(mrc_client_connection_t con, mrc_pose * pose, struct timeval *tv);
  extern int mrc_getodo_vel(mrc_client_connection_t con, mrc_pose_vel * posevel, struct timeval *tv);
  extern int mrc_getbattstat(mrc_client_connection_t con, mrc_batt_stat_t * battstat, struct timeval *tv);
  extern int mrc_getrks_vel(mrc_client_connection_t con, mrc_vel * vel, struct timeval *tv);
  extern int mrc_setpose(mrc_client_connection_t con, mrc_pose pose);
  extern int mrc_setallpose(mrc_client_connection_t con, mrc_pose pose);
  extern int mrc_setposedelta(mrc_client_connection_t con, mrc_pose pose);
  extern int mrc_getstatus(mrc_client_connection_t con, mrc_mode * mode, mrc_stat_gyro * gyro, mrc_stat * stat);
  extern int mrc_get_localization_status(mrc_client_connection_t con, mrc_localization_status * status);
  extern int mrc_set_localization_status(mrc_client_connection_t con, const mrc_localization_status status);
  extern int mrc_getgyrodata(mrc_client_connection_t con, mrc_gyrodata * data);
  extern int mrc_getwheelspeed(mrc_client_connection_t con, mrc_wheelspeed * data);
#if !defined(SWIGJAVASCRIPT)
  extern int mrc_addcallback(mrc_client_connection_t con, mrc_callback_func func, mrc_callback_type type, unsigned int period);
  extern int mrc_addcallback_ud(mrc_client_connection_t con, mrc_callback_func_ud func, mrc_callback_type type, unsigned int period, void *userdata);
  extern int mrc_removecallback(mrc_client_connection_t con, mrc_callback_func func, mrc_callback_type type);
  extern int mrc_removecallback_ud(mrc_client_connection_t con, mrc_callback_func_ud func, mrc_callback_type type, void *userdata);
#endif
  extern int mrc_netdispatcher(mrc_client_connection_t con);
  extern int mrc_sendcourse(mrc_client_connection_t con, mrc_interpol_course_t * course);
  extern void *mrc_netdispatcher_thread(mrc_client_connection_t con, void *lock);
  extern int mrc_getlaserdata(mrc_client_connection_t con, mrc_laserdata_t* data);
  extern int mrc_getalllaserdata(mrc_client_connection_t con, mrc_laserdatatype_t types, mrc_laserdata_t* dst_laserdata, mrc_laserreflectordata_t* dst_reflectordata, mrc_lasersegmentdata_t* dst_segmentdata);
  extern int mrc_getconfiguredlaser(mrc_client_connection_t con, mrc_configuredlaser_t* data);
  extern mrc_course_file_t *mrc_getcoursefile_list(mrc_client_connection_t con, const char *dir, int *count);
  extern int mrc_getcoursefile(mrc_client_connection_t con, const char *filename, mrc_interpol_course_t* course);
  extern int mrc_savecoursefile(mrc_client_connection_t con, const char *filename, mrc_interpol_course_t* course);
  extern int mrc_deletecoursefile(mrc_client_connection_t con, const char *filename);
  extern char *mrc_get_errmsg(mrc_client_connection_t con);
  extern int mrc_login(mrc_client_connection_t con, const char *username, const char *password);
  extern long int mrc_ping(mrc_client_connection_t con, mrc_ping_dst dst, struct timeval *dst_time);
  extern int mrc_getmotorcurrent(mrc_client_connection_t con, mrc_motor_current * data);
  extern int mrc_getrfiddata(mrc_client_connection_t con, mrc_rfid_scandata_t* dst_rfiddata);
  extern int mrc_getrfidconfig(mrc_client_connection_t con, mrc_rfid_cfg_scanner_t* data);
  extern int mrc_setinterpolmode(mrc_client_connection_t con, mrc_interpol_mode_t mode);
  extern int mrc_getvectormapfile(mrc_client_connection_t con, const char *filename, mrc_vectormap_t* vectormap);
  extern mrc_vectormap_file_t *mrc_vectormap_file_list(mrc_client_connection_t con, const char *dir, int *count);
  extern int mrc_getmotordata(mrc_client_connection_t con, mrc_motor_current *motorcurrent, mrc_motor_actual_pwm_t *motorpwm, mrc_wheelspeed *wheelspeed);
  extern const char *mrc_get_status_type_str(mrc_status_type_t id);
  extern const char *mrc_get_interpol_state_str(int id);
  extern const char *mrc_get_interpol_err_str(int id);
  extern const char *mrc_get_stat_gyro_str(mrc_stat_gyro id);
  extern const char *mrc_get_stat_str(mrc_stat id);
  extern int mrc_resetnotaus(mrc_client_connection_t con);
  extern mrc_landmarks_file_t *mrc_getlandmarksfile_list(mrc_client_connection_t con, const char *dir, int *count);
  extern int mrc_getlandmarksfile(mrc_client_connection_t con, const char *filename, mrc_landmarks_t* landmarks);
  extern int mrc_savelandmarksfile(mrc_client_connection_t con, const char *filename, mrc_landmarks_t* landmarks);
  extern int mrc_deletelandmarksfile(mrc_client_connection_t con, const char *filename);
  extern int mrc_actuator_getlist(mrc_client_connection_t con, mrc_actuator_list_t *list);
  extern int mrc_actuator_getstatus(mrc_client_connection_t con, mrc_actuator_id_t id, mrc_actuator_status_t *status);
  extern int mrc_actuator_getdist(mrc_client_connection_t con, mrc_actuator_id_t id, mrc_actuator_position_t *dist);
  extern int mrc_actuator_setdist(mrc_client_connection_t con, mrc_actuator_id_t id, mrc_actuator_position_t dist);
  extern int mrc_actuator_reference(mrc_client_connection_t con, uint8_t all);
  extern int mrc_actuator_getconfig(mrc_client_connection_t con, mrc_actuator_id_t id, mrc_actuator_config_t *config);
  extern int mrc_actuator_setconfig(mrc_client_connection_t con, mrc_actuator_id_t id, mrc_actuator_config_t config);
  extern int mrc_getbattstat_ext(mrc_client_connection_t con, mrc_batt_stat_ext_t * stat, struct timeval *tv);
  extern int mrc_getcurinterpolsegnr(mrc_client_connection_t con, uint32_t *segnr);
  extern mrc_svc_prog_t *mrc_getsvcprogstat_list(mrc_client_connection_t con, int *count);
  extern int mrc_svcprog_start(mrc_client_connection_t con, uint8_t prog_nr);
  extern int mrc_svcprog_stop(mrc_client_connection_t con, uint8_t prog_nr);
  extern int mrc_drivetovertex(mrc_client_connection_t con, uint32_t dst_vertex);
  extern int mrc_update_ecef_corners(mrc_client_connection_t con, mrc_position_3d_t p0, mrc_position_3d_t px, mrc_position_3d_t py);

#if __cplusplus
}
#endif

#endif				/* MRCLIENT_H */
