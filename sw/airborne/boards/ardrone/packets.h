#ifndef BOARDS_ARDRONE_PACKETS_H
#define BOARDS_ARDRONE_PACKETS_H

// Define constants for gyrometers handling
typedef enum {
	GYRO_X    = 0,
	GYRO_Y    = 1,
	GYRO_Z    = 2,
	NB_GYROS  = 3
} def_gyro_t;

// Define constants for accelerometers handling
typedef enum {
	ACC_X   = 0,
	ACC_Y   = 1,
	ACC_Z   = 2,
	NB_ACCS = 3
} def_acc_t;

typedef enum _navdata_tag_t {
	NAVDATA_DEMO_TAG = 0,
	NAVDATA_TIME_TAG
} navdata_tag_t;

typedef struct _navdata_option_t {
	uint16_t  tag;
	uint16_t  size;
	uint8_t   data[1];
} __attribute__ ((packed)) navdata_option_t;

typedef struct _navdata_t {
	uint32_t    header;			/*!< Always set to NAVDATA_HEADER */
	uint32_t    ardrone_state;    /*!< Bit mask built from def_ardrone_state_mask_t */
	uint32_t    sequence;         /*!< Sequence number, incremented for each sent packet */
	uint32_t    vision_defined;

	navdata_option_t  options[1];
} __attribute__ ((packed)) navdata_t;

typedef struct _navdata_cks_t {
	uint16_t  tag;
	uint16_t  size;
	uint32_t  cks;
} __attribute__ ((packed)) navdata_cks_t;

typedef struct _navdata_demo_t {
	uint16_t			tag;					/*!< Navdata block ('option') identifier */
	uint16_t			size;					/*!< set this to the size of this structure */
	uint32_t			ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
	uint32_t			vbat_flying_percentage; /*!< battery voltage filtered (mV) */
	float				theta;                  /*!< UAV's pitch in milli-degrees */
	float				phi;                    /*!< UAV's roll  in milli-degrees */
	float				psi;                    /*!< UAV's yaw   in milli-degrees */
	int32_t				altitude;               /*!< UAV's altitude in centimeters */
	float				vx;                     /*!< UAV's estimated linear velocity */
	float				vy;                     /*!< UAV's estimated linear velocity */
	float				vz;                     /*!< UAV's estimated linear velocity */
	uint32_t			num_frames;			    /*!< streamed frame index */ // Not used -> To integrate in video stage.
	// Camera parameters compute by detection
	struct FloatMat33	detection_camera_rot;   /*!<  Deprecated ! Don't use ! */
	struct FloatVect3	detection_camera_trans; /*!<  Deprecated ! Don't use ! */
	uint32_t			detection_tag_index;    /*!<  Deprecated ! Don't use ! */
	uint32_t	  		detection_camera_type;  /*!<  Type of tag searched in detection */
	// Camera parameters compute by drone
	struct FloatMat33	drone_camera_rot;		/*!<  Deprecated ! Don't use ! */
	struct FloatVect3	drone_camera_trans;	    /*!<  Deprecated ! Don't use ! */
} __attribute__ ((packed)) navdata_demo_t;


#endif /* BOARDS_ARDRONE_PACKETS_H */
