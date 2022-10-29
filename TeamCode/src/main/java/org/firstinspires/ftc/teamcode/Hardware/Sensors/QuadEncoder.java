package org.firstinspires.ftc.teamcode.hardware.Sensors;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

/*
 * Created by Dryw Wade
 *
 * Driver for Adafruit's MCP9808 temperature sensor
 *
 * This version of the driver does not make use of the I2C device with parameters. This means the
 * settings for the configuration register are hard coded and cannot be changed by the user, nor can
 * they be different for each OpMode.
 */
@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings
// Both driver classes cannot register the sensor at the same time. One driver should have the
// sensor registered, and the other should be commented out
@I2cDeviceType
@DeviceProperties(name = "Quadrature Encoder board", description = "an I2C interface for connecting to quadrature encoders", xmlTag = "QuadEnc")
public class QuadEncoder extends I2cDeviceSynchDevice<I2cDeviceSynch> {
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // User Methods
  ////////////////////////////////////////////////////////////////////////////////////////////////
public short getQuadrature(int pin){
  return TypeConversion.byteArrayToShort(deviceClient.read(pin));
}


  public short getManufacturerIDRaw()
  {
    return readShort(Register.MANUFACTURER_ID);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Read and Write Methods
  ////////////////////////////////////////////////////////////////////////////////////////////////

  protected void writeShort(final Register reg, short value)
  {
    deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
  }

//  protected void writeShort(byte[] value)
//  {
//    deviceClient.write(value);
//  }

  protected short readShort(Register reg)
  {
    return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Registers and Config Settings
  ////////////////////////////////////////////////////////////////////////////////////////////////

  public enum Register
  {
    FIRST(0),
    CONFIGURATION(0x01),
    T_LIMIT_UPPER(0x02),
    T_LIMIT_LOWER(0x03),
    T_LIMIT_CRITICAL(0x04),
    TEMPERATURE(0x05),
    MANUFACTURER_ID(0x06),
    DEVICE_ID_REVISION(0x07),
    RESOLUTION(0x08),
    LAST(RESOLUTION.bVal);

    public int bVal;

    Register(int bVal)
    {
      this.bVal = bVal;
    }
  }

  public enum Hysteresis
  {
    HYST_0(0x0000),
    HYST_1_5(0x0200),
    HYST_3(0x0400),
    HYST_6(0x0600);

    public int bVal;

    Hysteresis(int bVal)
    {
      this.bVal = bVal;
    }
  }

  public enum AlertControl
  {
    ALERT_DISABLE(0x0000),
    ALERT_ENABLE(0x0008);

    public int bVal;

    AlertControl(int bVal)
    {
      this.bVal = bVal;
    }
  }

  // More settings are available on the sensor, but not included here. Could be added later

  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Construction and Initialization
  ////////////////////////////////////////////////////////////////////////////////////////////////

  public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x18);

  public QuadEncoder(I2cDeviceSynch deviceClient)
  {
    super(deviceClient, true);

    this.setOptimalReadWindow();
    this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

    super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
    // Sensor starts off disengaged so we can change things like I2C address. Need to engage
    this.deviceClient.engage();
  }

  protected void setOptimalReadWindow()
  {
    // Sensor registers are read repeatedly and stored in a register. This method specifies the
    // registers and repeat read mode
    I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
      Register.FIRST.bVal,
      Register.LAST.bVal - Register.FIRST.bVal + 1,
      I2cDeviceSynch.ReadMode.REPEAT);
    this.deviceClient.setReadWindow(readWindow);
  }


//  void begin(byte pin, byte secondPin, byte debounce_mS, boolean pullUpsEnabled)
//  {
////    _pin = pin;
////    _secondPin = secondPin;
//    int PIN_MODE_QUADRATUREENCODER = 5;
////    byte[] tx = {(byte) 200, pin, (byte) PIN_MODE_QUADRATUREENCODER,debounce_mS, secondPin,(byte) QE_ONBOTH_INT,pullUpsEnabled };
////    short[] rx = new short[8];
//    writeShort(new byte[]{(byte) 200, pin, (byte) PIN_MODE_QUADRATUREENCODER, debounce_mS, secondPin, (byte) QE_ONBOTH_INT, (byte) (pullUpsEnabled?1:0)});
////    _sw.sendPacket(tx, rx);
//  }
  @Override
  protected synchronized boolean doInitialize(){
    byte pin = 0,
      secondPin = 1,
      debounce_mS = 0;
    boolean pullUpsEnabled = false;
    int PIN_MODE_QUADRATUREENCODER = 5;
//    byte[] tx = {(byte) 200, pin, (byte) PIN_MODE_QUADRATUREENCODER,debounce_mS, secondPin,(byte) QE_ONBOTH_INT,pullUpsEnabled };
//    short[] rx = new short[8];
    deviceClient.write(
      new byte[]{(byte)
        200,
        pin,
        (byte) PIN_MODE_QUADRATUREENCODER,
        debounce_mS,
        secondPin,
        (byte) QE_ONBOTH_INT,
        (byte) 0 /*pull ups enabled == 1*/},
        I2cWaitControl.ATOMIC);

//    int configSettings = Hysteresis.HYST_1_5.bVal | AlertControl.ALERT_ENABLE.bVal;

//    writeShort(Register.CONFIGURATION, (short) configSettings);

    // Mask out alert signal bit, which we can't control
//    return (readShort(Register.CONFIGURATION) & 0xFFEF) == configSettings;
    return true;
  }

  @Override
  public Manufacturer getManufacturer()
  {
    return Manufacturer.Adafruit;
  }

  @Override
  public String getDeviceName()
  {
    return "Adafruit MCP9808 Temperature Sensor";
  }



    int QE_ONLOW_INT = 0,  ///< Interrupt driven, process on high to low transition
    QE_ONHIGH_INT = 1, ///< Interrupt driven, process on low to high transition
    QE_ONBOTH_INT = 2, ///< Interrupt driven, process on low to high and high to low transition
    QE_ONLOW_POLL = 4,   ///< 1mS Polling, process on high to low transition
    QE_ONHIGH_POLL = 5,	 ///< 1mS Polling, process on low to high transition
    QE_ONBOTH_POLL = 6;	 ///< 1mS Polling, process on low to high and high to low transition


}
