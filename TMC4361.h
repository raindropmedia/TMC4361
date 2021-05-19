/*
 * TMC4361 Motion control IC by Trinamic.
 *
 * Note that this library doesn't provide a clock to the TMC4361 as this is platform-dependent.
 *
 * Tom Magnier <tom@tmagnier.fr> 08/2016
 */

#ifndef TMC4361_H
#define TMC4361_H

#include "Arduino.h"
#include "SPI.h"

//registers for TMC4361 (from https://github.com/trinamic/T-Bone/blob/master/Software/ArduinoClient/constants.h)
#define TMC4361_GENERAL_CONFIG_REGISTER 0x0
#define TMC4361_REFERENCE_CONFIG_REGISTER 0x01
#define TMC4361_START_CONFIG_REGISTER 0x2
#define TMC4361_INPUT_FILTER_REGISTER 0x3
#define TMC4361_SPIOUT_CONF_REGISTER 0x04
#define TMC4361_CURRENT_CONF_REGISTER 0x05
#define TMC4361_SCALE_VALUES_REGISTER 0x06
#define TMC4361_ENCODER_INPUT_CONFIG_REGISTER 0x07
#define TMC4361_ENC_IN_DATA 0x08
#define TMC4361_ENC_OUT_DATA 0x09
#define TMC4361_STEP_CONF_REGISTER 0x0A
#define TMC4361_SPI_STATUS_SELECTION 0x0B
#define TMC4361_EVENT_CLEAR_CONF_REGISTER 0x0C
#define TMC4361_INTERRUPT_CONFIG_REGISTER 0x0D
#define TMC4361_EVENTS_REGISTER 0x0E
#define TMC4361_STATUS_REGISTER 0x0F
#define TMC4361_STP_LENGTH_ADD 0x10
#define TMC4361_START_OUT_ADD_REGISTER 0x11
#define TMC4361_GEAR_RATIO_REGISTER 0x12
#define TMC4361_START_DELAY_REGISTER 0x13
#define TMC4361_STDBY_DELAY_REGISTER 0x15
#define TMC4361_FREEWHEEL_DELAY_REGISTER 0x16
#define TMC4361_VRDV_SCALE_LIMIT_REGISTER 0x17
#define TMC4361_UP_SCALE_DELAY_REGISTER 0x18
#define TMC4361_HOLD_SCALE_DELAY_REGISTER 0x19
#define TMC4361_DRV_SCALE_DELAY_REGISTER 0x1A
#define TMC4361_BOOST_TIME_REGISTER 0x1B
#define TMC4361_CLOSE_LOOP_REGISTER 0x1C
#define TMC4361_DAC_ADDR_REGISTER 0x1D
#define TMC4361_HOME_SAFETY_MARGIN_REGISTER 0x1E
#define TMC4361_PWM_FREQ_CHOPSYNC_REGISTER 0x
#define TMC4361_RAMP_MODE_REGISTER 0x20
#define TMC4361_X_ACTUAL_REGISTER 0x21
#define TMC4361_V_ACTUAL_REGISTER 0x22
#define TMC4361_A_ACTUAL_REGISTER 0x23
#define TMC4361_V_MAX_REGISTER 0x24
#define TMC4361_V_START_REGISTER 0x25
#define TMC4361_V_STOP_REGISTER 0x26
#define TMC4361_V_BREAK_REGISTER 0x27
#define TMC4361_A_MAX_REGISTER 0x28
#define TMC4361_D_MAX_REGISTER 0x29
#define TMC4361_A_START_REGISTER 0x2A
#define TMC4361_D_FINAL_REGISTER 0x2B
#define TMC4361_D_STOP_REGISTER 0x2C
#define TMC4361_BOW_1_REGISTER 0x2D
#define TMC4361_BOW_2_REGISTER 0x2E
#define TMC4361_BOW_3_REGISTER 0x2F
#define TMC4361_BOW_4_REGISTER 0x30
#define TMC4361_CLK_FREQ_REGISTER 0x31
#define TMC4361_POSITION_COMPARE_REGISTER 0x32
#define TMC4361_VIRTUAL_STOP_LEFT_REGISTER 0x33
#define TMC4361_VIRTUAL_STOP_RIGHT_REGISTER 0x34
#define TMC4361_X_HOME_REGISTER 0x35
#define TMC4361_X_LATCH_REGISTER 0x36
#define TMC4361_X_TARGET_REGISTER 0x37
#define TMC4361_X_TARGET_PIPE_0_REGSISTER 0x38
#define TMC4361_SH_V_MAX_REGISTER 0x40
#define TMC4361_SH_A_MAX_REGISTER 0x41
#define TMC4361_SH_D_MAX_REGISTER 0x42
#define TMC4361_SH_VBREAK_REGISTER 0x45
#define TMC4361_SH_V_START_REGISTER 0x46
#define TMC4361_SH_V_STOP_REGISTER 0x47
#define TMC4361_SH_BOW_1_REGISTER 0x48
#define TMC4361_SH_BOW_2_REGISTER 0x49
#define TMC4361_SH_BOW_3_REGISTER 0x4A
#define TMC4361_SH_BOW_4_REGISTER 0x4B
#define TMC4361_SH_RAMP_MODE_REGISTER 0x4C
#define TMC4361_D_FREEZE_REGISTER 0x4E
#define TMC4361_RESET_CLK_GATING_REGISTER 0x4F
#define TMC4361_ENCODER_POSITION_REGISTER 0x50
#define TMC4361_ENCODER_INPUT_RESOLUTION_REGISTER 0x54
#define TMC4361_COVER_LOW_REGISTER 0x6c
#define TMC4361_COVER_HIGH_REGISTER 0x6d
#define TMC4361_VERSION_REGISTER 0x7f

class TMC4361
{
public:
  enum RampMode {
    VELOCITY_MODE = 0x00,
    POSITIONING_MODE = (0x01 << 2)
  };

  enum RampType {
    HOLD_RAMP = 0x00, //Follow max speed (rectangle shape)
    TRAPEZOIDAL_RAMP = 0x01,
    S_SHAPED_RAMP = 0x02
  };

  //See Status Events Register description for details
  enum EventType {
    TARGET_REACHED = 0,
    POS_COMP_REACHED,
    VEL_REACHED,
    VEL_STATE_ZERO,
    VEL_STATE_POS,
    VEL_STATE_NEG,
    RAMP_STATE_ACCEL_ZERO,
    RAMP_STATE_ACCEL_POS,
    RAMP_STATE_ACCEL_NEG,
    MAX_PHASE_TRAP,
    FROZEN,
    STOPL = 11,
    STOPR = 12,
    VSTOPL_ACTIVE = 13,
    VSTOPR_ACTIVE = 14,
    HOME_ERROR = 15,
    XLATCH_DONE = 16,
    FS_ACTIVE,
    ENC_FAIL,
    N_ACTIVE,
    ENC_DONE,
    SER_ENC_DATA_FAIL,
    SER_DATA_DONE = 23,
    SERIAL_ENC_FLAG,
    COVER_DONE,
    ENC_VEL_ZERO,
    CL_MAX,
    CL_FIT,
    STOP_ON_STALL_EV,
    MOTOR_EV,
    RST_EV
  };

  //See Status Flags Register description for details
  enum FlagType {
    TARGET_REACHED_F = 0,
    POS_COMP_REACHED_F,
    VEL_REACHED_F,
    VEL_STATE_F0,
    VEL_STATE_F1,
    RAMP_STATE_F0,
    RAMP_STATE_F1,
    STOPL_ACTIVE_F,
    STOPR_ACTIVE_F,
    VSTOPL_ACTIVE_F,
    VSTOPR_ACTIVE_F,
    ACTIVE_STALL_F,
    HOME_ERROR_F,
    FS_ACTIVE_F,
    ENC_FAIL_F,
    N_ACTIVE_F,
    ENC_LATCH_F,
  };
  
  //Reference switch configuration register
  enum ReferenceConfRegisterFields {
    STOP_LEFT_EN = 0,
    STOP_RIGHT_EN,
    POL_STOP_LEFT,
    POL_STOP_RIGHT,
    INVERT_STOP_DIRECTION,
    SOFT_STOP_EN,
    VIRTUAL_LEFT_LIMIT_EN,
    VIRTUAL_RIGHT_LIMIT_EN,
    VIRT_STOP_MODE,
    LATCH_X_ON_INACTIVE_L = 10,
    LATCH_X_ON_ACTIVE_L,
    LATCH_X_ON_INACTIVE_R,
    LATCH_X_ON_ACTIVE_R,
    STOP_LEFT_IS_HOME,
    STOP_RIGHT_IS_HOME,
    HOME_EVENT,
    START_HOME_TRACKING = 20,
    CLR_POS_AT_TARGET,
    CIRCULAR_MOVEMENT_EN,
    POS_COMP_OUTPUT,
    POS_COMP_SOURCE = 25,
    STOP_ON_STALL,
    DRV_AFTER_STALL,
    MODIFIED_POS_COMPARE,
    AUTOMATIC_COVER = 30,
    CIRCULAR_ENC_EN
  };

  TMC4361();
  void begin(long clockFreq, int csPin);
  void begin(long clockFreq, int csPin, int intPin);
  void begin(long clockFreq, int csPin, int intPin, int startPin);
  void begin(long clockFreq, int csPin, int intPin, int startPin, int rstPin);

  /* HW or SW reset */
  void reset();

  /* Check runtime flags (as-is condition, as opposed to events that indicate a change since the last read) */
  bool checkFlag(FlagType flag);
  bool isTargetReached();

  //TODO interrupt configuration
  
  /* Clear all events */
  void clearEvents();
  
  /* Check event and clear it if necessary */
  bool checkEvent(EventType event);

  /* Set step/dir outputs polarity
   * if stepInverted is true, LOW indicates an active step
   * if dirInverted is true, HIGH indicates negative direction
   */
  void setOutputsPolarity(bool stepInverted, bool dirInverted);

  /* Set output interface timings :
   * step duration in microseconds
   * dir setup time in microseconds (no step will be issued for this duration after a direction change)
   */
  void setOutputTimings(int stepWidth, int dirSetupTime);

  /* Ramp generator commands */
  void setRampMode(RampMode mode, RampType type);

  /* Return the current internal position (in steps) */
  long getCurrentPosition();

  /* Set the current internal position (in steps) */
  void setCurrentPosition(long position);
  
  /* Return the current Left Virtual Limit position (in steps) */
  long getLeftVirtualLimit();
  
  /* Set the current Left Virtual Limit position (in steps) */
  void setLeftVirtualLimit(long position);
  
  /* Return the current Right Virtual Limit position (in steps) */
  long getRightVirtualLimit();
  
  /* Set the current Right Virtual Limit position (in steps) */
  void setRightVirtualLimit(long position);

  /* Return the current speed (in steps / second) */
  float getCurrentSpeed();

  /* Return the current acceleration (in steps / second^2) */
  float getCurrentAcceleration();

  /* Set the max speed VMAX (steps/second)
   * /!\ Don't exceed clockFreq / 2 in velocity mode and clockFreq / 4 in positioning mode
   */
  void setMaxSpeed(float speed);

  /* Set the ramp start, stop and break speeds (in steps / second). See datasheet §6.4 for details */
  void setRampSpeeds(float startSpeed, float stopSpeed, float breakSpeed);

  /* Set the ramp accelerations (in steps / second^2). See datasheet §6.3.6 */
  void setAccelerations(float maxAccel, float maxDecel, float startAccel, float finalDecel);

  /* Set the bow values for S-shaped ramps (in steps / second^3). */
  void setBowValues(long bow1, long bow2, long bow3, long bow4);

  /* Get the target position in steps */
  long getTargetPosition();

  /* Set the target position
   * /!\ Set all other motion profile parameters before
   */
  void setTargetPosition(long position);

  /* Stop the current motion according to the set ramp mode and motion parameters. The max speed is set to 0 but the target position stays unchanged. */
  void stop();

  void writeRegister(const byte address, const long data);
  long readRegister(const byte address);
  
  void setRegisterBit(const byte address, const byte bit);
  void clearRegisterBit(const byte address, const byte bit);
  bool readRegisterBit(const byte address, const byte bit);

private:
  const static int _defaultStepLength = 5; //us
  const static int _defaultDirSetupTime = 5; //us

  long _clockFreq; //TMC4361 clock frequency (Hz)
  int _csPin; //Chip Select pin number
  int _startPin; //Start signal pin number
  int _intPin; //Interrupt line pin number
  int _rstPin; //Reset pin number

  byte _spiStatus; //Contents of the status bits SPI_STATUS updated on each SPI transaction

  SPISettings _spiSettings;

  long spiTransfer(const byte address, const long data);
  long floatToFixedPoint(float value, int decimalPlaces);
  float fixedPointToFloat(long value, int decimalPlaces);
};

#endif //TMC4361_H
