/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package com.ctre.phoenix6.hardware.core;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.compound.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.traits.*;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;
import java.util.HashMap;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

/**
 * Class description for the Talon FX integrated motor controller.
 * 
 * <pre>
 * // Constants used in TalonFX construction
 * final int kTalonFXId = 0;
 * final CANBus kTalonFXCANbus = new CANBus("canivore");
 * 
 * // Construct the TalonFX and control requests
 * final TalonFX talonfx = new TalonFX(kTalonFXId, kTalonFXCANbus);
 * final DutyCycleOut dc = new DutyCycleOut(0);
 * 
 * // Configure the TalonFX for basic use
 * TalonFXConfiguration configs = new TalonFXConfiguration();
 * // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
 * configs.Slot0.kP = 1;
 * configs.Slot0.kI = 0;
 * configs.Slot0.kD = 10;
 * configs.Slot0.kV = 2;
 * 
 * // Write these configs to the TalonFX
 * talonfx.getConfigurator().apply(configs);
 * 
 * // Set the position to 0 rotations for initial use
 * talonfx.setPosition(0);
 * 
 * // Drive at 50% output
 * talonfx.setControl(dc.withOutput(0.5));
 * 
 * // Get Position and Velocity
 * var position = talonfx.getPosition();
 * var velocity = talonfx.getVelocity();
 * 
 * // Refresh and print these values
 * System.out.println("Position is " + position.refresh().toString());
 * System.out.println("Velocity is " + velocity.refresh().toString());
 * </pre>
 */
public class CoreTalonFX extends ParentDevice implements CommonTalonWithFOC
{
    private TalonFXConfigurator _configurator;

    /**
     * Constructs a new Talon FX motor controller object.
     * <p>
     * Constructs the device using the default CAN bus for the system
     * (see {@link CANBus#CANBus()}).
     *
     * @param deviceId    ID of the device, as configured in Phoenix Tuner
     */
    public CoreTalonFX(int deviceId)
    {
        this(deviceId, new CANBus());
    }

    /**
     * Constructs a new Talon FX motor controller object.
     *
     * @param deviceId    ID of the device, as configured in Phoenix Tuner
     * @param canbus      Name of the CAN bus this device is on. Possible CAN bus strings are:
     *                    <ul>
     *                      <li>"rio" for the native roboRIO CAN bus
     *                      <li>CANivore name or serial number
     *                      <li>SocketCAN interface (non-FRC Linux only)
     *                      <li>"*" for any CANivore seen by the program
     *                      <li>empty string (default) to select the default for the system:
     *                      <ul>
     *                        <li>"rio" on roboRIO
     *                        <li>"can0" on Linux
     *                        <li>"*" on Windows
     *                      </ul>
     *                    </ul>
     *
     * @deprecated Constructing devices with a CAN bus string is deprecated for removal
     * in the 2027 season. Construct devices using a {@link CANBus} instance instead.
     */
    @Deprecated(since = "2026", forRemoval = true)
    public CoreTalonFX(int deviceId, String canbus)
    {
        this(deviceId, new CANBus(canbus));
    }

    /**
     * Constructs a new Talon FX motor controller object.
     *
     * @param deviceId    ID of the device, as configured in Phoenix Tuner
     * @param canbus      The CAN bus this device is on
     */
    public CoreTalonFX(int deviceId, CANBus canbus)
    {
        super(deviceId, "talon fx", canbus);
        _configurator = new TalonFXConfigurator(this.deviceIdentifier);
        PlatformJNI.JNI_SimCreate(DeviceType.P6_TalonFXType.value, deviceId);
    }

    /**
     * Gets the configurator to use with this device's configs
     *
     * @return Configurator for this object
     */
    public final TalonFXConfigurator getConfigurator()
    {
        return this._configurator;
    }


    private TalonFXSimState _simState = null;
    /**
     * Get the simulation state for this device.
     * <p>
     * This function reuses an allocated simulation state
     * object, so it is safe to call this function multiple
     * times in a robot loop.
     *
     * @return Simulation state
     */
    public final TalonFXSimState getSimState() {
        if (_simState == null)
            _simState = new TalonFXSimState(this);
        return _simState;
    }


        
    /**
     * App Major Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return VersionMajor Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionMajor()
    {
        return getVersionMajor(true);
    }
    
    /**
     * App Major Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return VersionMajor Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionMajor(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Version_Major.value, "VersionMajor", Integer.class, val -> (int)val, false, refresh);
        return retval;
    }
        
    /**
     * App Minor Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return VersionMinor Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionMinor()
    {
        return getVersionMinor(true);
    }
    
    /**
     * App Minor Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return VersionMinor Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionMinor(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Version_Minor.value, "VersionMinor", Integer.class, val -> (int)val, false, refresh);
        return retval;
    }
        
    /**
     * App Bugfix Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return VersionBugfix Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionBugfix()
    {
        return getVersionBugfix(true);
    }
    
    /**
     * App Bugfix Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return VersionBugfix Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionBugfix(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Version_Bugfix.value, "VersionBugfix", Integer.class, val -> (int)val, false, refresh);
        return retval;
    }
        
    /**
     * App Build Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return VersionBuild Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionBuild()
    {
        return getVersionBuild(true);
    }
    
    /**
     * App Build Version number.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 255
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return VersionBuild Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersionBuild(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Version_Build.value, "VersionBuild", Integer.class, val -> (int)val, false, refresh);
        return retval;
    }
        
    /**
     * Full Version of firmware in device.  The format is a four byte
     * value.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 4294967295
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Version Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersion()
    {
        return getVersion(true);
    }
    
    /**
     * Full Version of firmware in device.  The format is a four byte
     * value.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 4294967295
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Version Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getVersion(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Version_Full.value, "Version", Integer.class, val -> (int)val, false, refresh);
        return retval;
    }
        
    /**
     * Integer representing all fault flags reported by the device.
     * <p>
     * These are device specific and are not used directly in typical
     * applications. Use the signal specific GetFault_*() methods instead.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 4294967295
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return FaultField Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getFaultField()
    {
        return getFaultField(true);
    }
    
    /**
     * Integer representing all fault flags reported by the device.
     * <p>
     * These are device specific and are not used directly in typical
     * applications. Use the signal specific GetFault_*() methods instead.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 4294967295
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return FaultField Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getFaultField(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.AllFaults.value, "FaultField", Integer.class, val -> (int)val, true, refresh);
        return retval;
    }
        
    /**
     * Integer representing all (persistent) sticky fault flags reported
     * by the device.
     * <p>
     * These are device specific and are not used directly in typical
     * applications. Use the signal specific GetStickyFault_*() methods
     * instead.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 4294967295
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFaultField Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getStickyFaultField()
    {
        return getStickyFaultField(true);
    }
    
    /**
     * Integer representing all (persistent) sticky fault flags reported
     * by the device.
     * <p>
     * These are device specific and are not used directly in typical
     * applications. Use the signal specific GetStickyFault_*() methods
     * instead.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 4294967295
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFaultField Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getStickyFaultField(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.AllStickyFaults.value, "StickyFaultField", Integer.class, val -> (int)val, true, refresh);
        return retval;
    }
        
    /**
     * The applied (output) motor voltage.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -40.96
     *   <li> <b>Maximum Value:</b> 40.95
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> V
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return MotorVoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Voltage> getMotorVoltage()
    {
        return getMotorVoltage(true);
    }
    
    /**
     * The applied (output) motor voltage.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -40.96
     *   <li> <b>Maximum Value:</b> 40.95
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> V
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return MotorVoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Voltage> getMotorVoltage(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_MotorVoltage.value, "MotorVoltage", Voltage.class, val -> Volts.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Forward Limit Pin.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ForwardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<ForwardLimitValue> getForwardLimit()
    {
        return getForwardLimit(true);
    }
    
    /**
     * Forward Limit Pin.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ForwardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<ForwardLimitValue> getForwardLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.ForwardLimit.value, "ForwardLimit", ForwardLimitValue.class, val -> ForwardLimitValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * Reverse Limit Pin.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ReverseLimit Status Signal Object
     */
    @Override
    public final StatusSignal<ReverseLimitValue> getReverseLimit()
    {
        return getReverseLimit(true);
    }
    
    /**
     * Reverse Limit Pin.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ReverseLimit Status Signal Object
     */
    @Override
    public final StatusSignal<ReverseLimitValue> getReverseLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.ReverseLimit.value, "ReverseLimit", ReverseLimitValue.class, val -> ReverseLimitValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * The applied rotor polarity as seen from the front of the motor. 
     * This typically is determined by the Inverted config, but can be
     * overridden if using Follower features.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return AppliedRotorPolarity Status Signal Object
     */
    @Override
    public final StatusSignal<AppliedRotorPolarityValue> getAppliedRotorPolarity()
    {
        return getAppliedRotorPolarity(true);
    }
    
    /**
     * The applied rotor polarity as seen from the front of the motor. 
     * This typically is determined by the Inverted config, but can be
     * overridden if using Follower features.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return AppliedRotorPolarity Status Signal Object
     */
    @Override
    public final StatusSignal<AppliedRotorPolarityValue> getAppliedRotorPolarity(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_RotorPolarity.value, "AppliedRotorPolarity", AppliedRotorPolarityValue.class, val -> AppliedRotorPolarityValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * The applied motor duty cycle.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -2.0
     *   <li> <b>Maximum Value:</b> 1.9990234375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> fractional
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DutyCycle Status Signal Object
     */
    @Override
    public final StatusSignal<Double> getDutyCycle()
    {
        return getDutyCycle(true);
    }
    
    /**
     * The applied motor duty cycle.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -2.0
     *   <li> <b>Maximum Value:</b> 1.9990234375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> fractional
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DutyCycle Status Signal Object
     */
    @Override
    public final StatusSignal<Double> getDutyCycle(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_DutyCycle.value, "DutyCycle", Double.class, val -> val, true, refresh);
        return retval;
    }
        
    /**
     * Current corresponding to the torque output by the motor. Similar to
     * StatorCurrent. Users will likely prefer this current to calculate
     * the applied torque to the rotor.
     * <p>
     * Stator current where positive current means torque is applied in
     * the forward direction as determined by the Inverted setting.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -327.68
     *   <li> <b>Maximum Value:</b> 327.67
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return TorqueCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getTorqueCurrent()
    {
        return getTorqueCurrent(true);
    }
    
    /**
     * Current corresponding to the torque output by the motor. Similar to
     * StatorCurrent. Users will likely prefer this current to calculate
     * the applied torque to the rotor.
     * <p>
     * Stator current where positive current means torque is applied in
     * the forward direction as determined by the Inverted setting.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -327.68
     *   <li> <b>Maximum Value:</b> 327.67
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return TorqueCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getTorqueCurrent(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_TorqueCurrent.value, "TorqueCurrent", Current.class, val -> Amps.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Current corresponding to the stator windings. Similar to
     * TorqueCurrent. Users will likely prefer TorqueCurrent over
     * StatorCurrent.
     * <p>
     * Stator current where Positive current indicates motoring regardless
     * of direction. Negative current indicates regenerative braking
     * regardless of direction.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -327.68
     *   <li> <b>Maximum Value:</b> 327.66
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StatorCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getStatorCurrent()
    {
        return getStatorCurrent(true);
    }
    
    /**
     * Current corresponding to the stator windings. Similar to
     * TorqueCurrent. Users will likely prefer TorqueCurrent over
     * StatorCurrent.
     * <p>
     * Stator current where Positive current indicates motoring regardless
     * of direction. Negative current indicates regenerative braking
     * regardless of direction.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -327.68
     *   <li> <b>Maximum Value:</b> 327.66
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StatorCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getStatorCurrent(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_SupplyAndTemp_StatorCurrent.value, "StatorCurrent", Current.class, val -> Amps.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Measured supply side current.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -327.68
     *   <li> <b>Maximum Value:</b> 327.66
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return SupplyCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getSupplyCurrent()
    {
        return getSupplyCurrent(true);
    }
    
    /**
     * Measured supply side current.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -327.68
     *   <li> <b>Maximum Value:</b> 327.66
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return SupplyCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getSupplyCurrent(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_SupplyAndTemp_SupplyCurrent.value, "SupplyCurrent", Current.class, val -> Amps.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Measured supply voltage to the device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 4
     *   <li> <b>Maximum Value:</b> 29.575
     *   <li> <b>Default Value:</b> 4
     *   <li> <b>Units:</b> V
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return SupplyVoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Voltage> getSupplyVoltage()
    {
        return getSupplyVoltage(true);
    }
    
    /**
     * Measured supply voltage to the device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 4
     *   <li> <b>Maximum Value:</b> 29.575
     *   <li> <b>Default Value:</b> 4
     *   <li> <b>Units:</b> V
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return SupplyVoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Voltage> getSupplyVoltage(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_SupplyAndTemp_SupplyVoltage.value, "SupplyVoltage", Voltage.class, val -> Volts.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Temperature of device.
     * <p>
     * This is the temperature that the device measures itself to be at.
     * Similar to Processor Temperature.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 255.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Temperature> getDeviceTemp()
    {
        return getDeviceTemp(true);
    }
    
    /**
     * Temperature of device.
     * <p>
     * This is the temperature that the device measures itself to be at.
     * Similar to Processor Temperature.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 255.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Temperature> getDeviceTemp(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_SupplyAndTemp_DeviceTemp.value, "DeviceTemp", Temperature.class, val -> Celsius.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Temperature of the processor.
     * <p>
     * This is the temperature that the processor measures itself to be
     * at. Similar to Device Temperature.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 255.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ProcessorTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Temperature> getProcessorTemp()
    {
        return getProcessorTemp(true);
    }
    
    /**
     * Temperature of the processor.
     * <p>
     * This is the temperature that the processor measures itself to be
     * at. Similar to Device Temperature.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 255.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ProcessorTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Temperature> getProcessorTemp(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_SupplyAndTemp_ProcessorTemp.value, "ProcessorTemp", Temperature.class, val -> Celsius.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Velocity of the motor rotor. This velocity is not affected by any
     * feedback configs.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return RotorVelocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getRotorVelocity()
    {
        return getRotorVelocity(true);
    }
    
    /**
     * Velocity of the motor rotor. This velocity is not affected by any
     * feedback configs.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return RotorVelocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getRotorVelocity(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_RotorPosAndVel_Velocity.value, "RotorVelocity", AngularVelocity.class, val -> RotationsPerSecond.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Position of the motor rotor. This position is only affected by the
     * RotorOffset config and calls to setPosition.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return RotorPosition Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getRotorPosition()
    {
        return getRotorPosition(true);
    }
    
    /**
     * Position of the motor rotor. This position is only affected by the
     * RotorOffset config and calls to setPosition.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return RotorPosition Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getRotorPosition(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_RotorPosAndVel_Position.value, "RotorPosition", Angle.class, val -> Rotations.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Velocity of the device in mechanism rotations per second. This can
     * be the velocity of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Velocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getVelocity()
    {
        return getVelocity(true);
    }
    
    /**
     * Velocity of the device in mechanism rotations per second. This can
     * be the velocity of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Velocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getVelocity(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PosAndVel_Velocity.value, "Velocity", AngularVelocity.class, val -> RotationsPerSecond.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Position of the device in mechanism rotations. This can be the
     * position of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs, as well as
     * calls to setPosition.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Position Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getPosition()
    {
        return getPosition(true);
    }
    
    /**
     * Position of the device in mechanism rotations. This can be the
     * position of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs, as well as
     * calls to setPosition.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Position Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getPosition(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PosAndVel_Position.value, "Position", Angle.class, val -> Rotations.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Acceleration of the device in mechanism rotations per second². This
     * can be the acceleration of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -2048.0
     *   <li> <b>Maximum Value:</b> 2047.75
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second²
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Acceleration Status Signal Object
     */
    @Override
    public final StatusSignal<AngularAcceleration> getAcceleration()
    {
        return getAcceleration(true);
    }
    
    /**
     * Acceleration of the device in mechanism rotations per second². This
     * can be the acceleration of a remote sensor and is affected by the
     * RotorToSensorRatio and SensorToMechanismRatio configs.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -2048.0
     *   <li> <b>Maximum Value:</b> 2047.75
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second²
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 50.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Acceleration Status Signal Object
     */
    @Override
    public final StatusSignal<AngularAcceleration> getAcceleration(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PosAndVel_Acceleration.value, "Acceleration", AngularAcceleration.class, val -> RotationsPerSecondPerSecond.of(val), true, refresh);
        return retval;
    }
        
    /**
     * The active control mode of the motor controller.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ControlMode Status Signal Object
     */
    @Override
    public final StatusSignal<ControlModeValue> getControlMode()
    {
        return getControlMode(true);
    }
    
    /**
     * The active control mode of the motor controller.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ControlMode Status Signal Object
     */
    @Override
    public final StatusSignal<ControlModeValue> getControlMode(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.TalonFX_ControlMode.value, "ControlMode", ControlModeValue.class, val -> ControlModeValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * Check if the Motion Magic® profile has reached the target. This is
     * equivalent to checking that MotionMagicIsRunning, the
     * ClosedLoopReference is the target, and the ClosedLoopReferenceSlope
     * is 0.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return MotionMagicAtTarget Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getMotionMagicAtTarget()
    {
        return getMotionMagicAtTarget(true);
    }
    
    /**
     * Check if the Motion Magic® profile has reached the target. This is
     * equivalent to checking that MotionMagicIsRunning, the
     * ClosedLoopReference is the target, and the ClosedLoopReferenceSlope
     * is 0.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return MotionMagicAtTarget Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getMotionMagicAtTarget(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDStateEnables_MotionMagicAtTarget.value, "MotionMagicAtTarget", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Check if Motion Magic® is running.  This is equivalent to checking
     * that the reported control mode is a Motion Magic® based mode.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return MotionMagicIsRunning Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getMotionMagicIsRunning()
    {
        return getMotionMagicIsRunning(true);
    }
    
    /**
     * Check if Motion Magic® is running.  This is equivalent to checking
     * that the reported control mode is a Motion Magic® based mode.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return MotionMagicIsRunning Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getMotionMagicIsRunning(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDStateEnables_IsMotionMagicRunning.value, "MotionMagicIsRunning", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Indicates if the robot is enabled.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return RobotEnable Status Signal Object
     */
    @Override
    public final StatusSignal<RobotEnableValue> getRobotEnable()
    {
        return getRobotEnable(true);
    }
    
    /**
     * Indicates if the robot is enabled.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return RobotEnable Status Signal Object
     */
    @Override
    public final StatusSignal<RobotEnableValue> getRobotEnable(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDStateEnables_RobotEnable.value, "RobotEnable", RobotEnableValue.class, val -> RobotEnableValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * Indicates if device is actuator enabled.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DeviceEnable Status Signal Object
     */
    @Override
    public final StatusSignal<DeviceEnableValue> getDeviceEnable()
    {
        return getDeviceEnable(true);
    }
    
    /**
     * Indicates if device is actuator enabled.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DeviceEnable Status Signal Object
     */
    @Override
    public final StatusSignal<DeviceEnableValue> getDeviceEnable(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDStateEnables_DeviceEnable.value, "DeviceEnable", DeviceEnableValue.class, val -> DeviceEnableValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * The slot that the closed-loop PID is using.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopSlot Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getClosedLoopSlot()
    {
        return getClosedLoopSlot(true);
    }
    
    /**
     * The slot that the closed-loop PID is using.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopSlot Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getClosedLoopSlot(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDOutput_Slot.value, "ClosedLoopSlot", Integer.class, val -> (int)val, true, refresh);
        return retval;
    }
        
    /**
     * Assess the status of the motor output with respect to load and
     * supply.
     * <p>
     * This routine can be used to determine the general status of motor
     * commutation.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return MotorOutputStatus Status Signal Object
     */
    @Override
    public final StatusSignal<MotorOutputStatusValue> getMotorOutputStatus()
    {
        return getMotorOutputStatus(true);
    }
    
    /**
     * Assess the status of the motor output with respect to load and
     * supply.
     * <p>
     * This routine can be used to determine the general status of motor
     * commutation.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return MotorOutputStatus Status Signal Object
     */
    @Override
    public final StatusSignal<MotorOutputStatusValue> getMotorOutputStatus(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.TalonFX_MotorOutputStatus.value, "MotorOutputStatus", MotorOutputStatusValue.class, val -> MotorOutputStatusValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * The active control mode of the differential controller.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialControlMode Status Signal Object
     */
    @Override
    public final StatusSignal<DifferentialControlModeValue> getDifferentialControlMode()
    {
        return getDifferentialControlMode(true);
    }
    
    /**
     * The active control mode of the differential controller.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialControlMode Status Signal Object
     */
    @Override
    public final StatusSignal<DifferentialControlModeValue> getDifferentialControlMode(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.TalonFX_DifferentialControlMode.value, "DifferentialControlMode", DifferentialControlModeValue.class, val -> DifferentialControlModeValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * Average component of the differential velocity of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialAverageVelocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getDifferentialAverageVelocity()
    {
        return getDifferentialAverageVelocity(true);
    }
    
    /**
     * Average component of the differential velocity of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialAverageVelocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getDifferentialAverageVelocity(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_AvgPosAndVel_Velocity.value, "DifferentialAverageVelocity", AngularVelocity.class, val -> RotationsPerSecond.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Average component of the differential position of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialAveragePosition Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getDifferentialAveragePosition()
    {
        return getDifferentialAveragePosition(true);
    }
    
    /**
     * Average component of the differential position of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialAveragePosition Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getDifferentialAveragePosition(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_AvgPosAndVel_Position.value, "DifferentialAveragePosition", Angle.class, val -> Rotations.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Difference component of the differential velocity of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialDifferenceVelocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getDifferentialDifferenceVelocity()
    {
        return getDifferentialDifferenceVelocity(true);
    }
    
    /**
     * Difference component of the differential velocity of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -512.0
     *   <li> <b>Maximum Value:</b> 511.998046875
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations per second
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialDifferenceVelocity Status Signal Object
     */
    @Override
    public final StatusSignal<AngularVelocity> getDifferentialDifferenceVelocity(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPosAndVel_Velocity.value, "DifferentialDifferenceVelocity", AngularVelocity.class, val -> RotationsPerSecond.of(val), true, refresh);
        return retval;
    }
        
    /**
     * Difference component of the differential position of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialDifferencePosition Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getDifferentialDifferencePosition()
    {
        return getDifferentialDifferencePosition(true);
    }
    
    /**
     * Difference component of the differential position of device.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> -16384.0
     *   <li> <b>Maximum Value:</b> 16383.999755859375
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rotations
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialDifferencePosition Status Signal Object
     */
    @Override
    public final StatusSignal<Angle> getDifferentialDifferencePosition(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPosAndVel_Position.value, "DifferentialDifferencePosition", Angle.class, val -> Rotations.of(val), true, refresh);
        return retval;
    }
        
    /**
     * The slot that the closed-loop differential PID is using.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopSlot Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getDifferentialClosedLoopSlot()
    {
        return getDifferentialClosedLoopSlot(true);
    }
    
    /**
     * The slot that the closed-loop differential PID is using.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 2
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopSlot Status Signal Object
     */
    @Override
    public final StatusSignal<Integer> getDifferentialClosedLoopSlot(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPIDOutput_Slot.value, "DifferentialClosedLoopSlot", Integer.class, val -> (int)val, true, refresh);
        return retval;
    }
        
    /**
     * The torque constant (K_T) of the motor.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 0.025500000000000002
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> Nm/A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return MotorKT Status Signal Object
     */
    @Override
    public final StatusSignal<Per<TorqueUnit, CurrentUnit>> getMotorKT()
    {
        return getMotorKT(true);
    }
    
    /**
     * The torque constant (K_T) of the motor.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 0.025500000000000002
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> Nm/A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return MotorKT Status Signal Object
     */
    @Override
    public final StatusSignal<Per<TorqueUnit, CurrentUnit>> getMotorKT(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.TalonFX_MotorConstants_kT.value, "MotorKT", (Class<Per<TorqueUnit, CurrentUnit>>)(Class<?>)Per.class, val -> NewtonMeters.per(Amp).ofNative(val), true, refresh);
        return retval;
    }
        
    /**
     * The velocity constant (K_V) of the motor.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 2047.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> RPM/V
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return MotorKV Status Signal Object
     */
    @Override
    public final StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> getMotorKV()
    {
        return getMotorKV(true);
    }
    
    /**
     * The velocity constant (K_V) of the motor.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 2047.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> RPM/V
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return MotorKV Status Signal Object
     */
    @Override
    public final StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> getMotorKV(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.TalonFX_MotorConstants_kV.value, "MotorKV", (Class<Per<AngularVelocityUnit, VoltageUnit>>)(Class<?>)Per.class, val -> RPM.per(Volt).ofNative(val), true, refresh);
        return retval;
    }
        
    /**
     * The stall current of the motor at 12 V output.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1023.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return MotorStallCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getMotorStallCurrent()
    {
        return getMotorStallCurrent(true);
    }
    
    /**
     * The stall current of the motor at 12 V output.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 1023.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> A
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return MotorStallCurrent Status Signal Object
     */
    @Override
    public final StatusSignal<Current> getMotorStallCurrent(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.TalonFX_MotorConstants_StallCurrent.value, "MotorStallCurrent", Current.class, val -> Amps.of(val), true, refresh);
        return retval;
    }
        
    /**
     * The applied output of the bridge.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return BridgeOutput Status Signal Object
     */
    @Override
    public final StatusSignal<BridgeOutputValue> getBridgeOutput()
    {
        return getBridgeOutput(true);
    }
    
    /**
     * The applied output of the bridge.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return BridgeOutput Status Signal Object
     */
    @Override
    public final StatusSignal<BridgeOutputValue> getBridgeOutput(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_BridgeType_Public.value, "BridgeOutput", BridgeOutputValue.class, val -> BridgeOutputValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * Whether the device is Phoenix Pro licensed.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return IsProLicensed Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getIsProLicensed()
    {
        return getIsProLicensed(true);
    }
    
    /**
     * Whether the device is Phoenix Pro licensed.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return IsProLicensed Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getIsProLicensed(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Version_IsProLicensed.value, "IsProLicensed", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Temperature of device from second sensor.
     * <p>
     * Newer versions of Talon have multiple temperature measurement
     * methods.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 255.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return AncillaryDeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Temperature> getAncillaryDeviceTemp()
    {
        return getAncillaryDeviceTemp(true);
    }
    
    /**
     * Temperature of device from second sensor.
     * <p>
     * Newer versions of Talon have multiple temperature measurement
     * methods.
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 255.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return AncillaryDeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Temperature> getAncillaryDeviceTemp(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_SupplyAndTemp_DeviceTemp2.value, "AncillaryDeviceTemp", Temperature.class, val -> Celsius.of(val), true, refresh);
        return retval;
    }
        
    /**
     * The type of motor attached to the Talon.
     * <p>
     * This can be used to determine what motor is attached to the Talon
     * FX.  Return will be "Unknown" if firmware is too old or device is
     * not present.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ConnectedMotor Status Signal Object
     */
    @Override
    public final StatusSignal<ConnectedMotorValue> getConnectedMotor()
    {
        return getConnectedMotor(true);
    }
    
    /**
     * The type of motor attached to the Talon.
     * <p>
     * This can be used to determine what motor is attached to the Talon
     * FX.  Return will be "Unknown" if firmware is too old or device is
     * not present.
     * 
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ConnectedMotor Status Signal Object
     */
    @Override
    public final StatusSignal<ConnectedMotorValue> getConnectedMotor(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.TalonFX_ConnectedMotor.value, "ConnectedMotor", ConnectedMotorValue.class, val -> ConnectedMotorValue.valueOf((int)val), true, refresh);
        return retval;
    }
        
    /**
     * Hardware fault occurred
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_Hardware Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_Hardware()
    {
        return getFault_Hardware(true);
    }
    
    /**
     * Hardware fault occurred
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_Hardware Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_Hardware(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_Hardware.value, "Fault_Hardware", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Hardware fault occurred
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_Hardware Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_Hardware()
    {
        return getStickyFault_Hardware(true);
    }
    
    /**
     * Hardware fault occurred
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_Hardware Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_Hardware(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_Hardware.value, "StickyFault_Hardware", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Processor temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_ProcTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ProcTemp()
    {
        return getFault_ProcTemp(true);
    }
    
    /**
     * Processor temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_ProcTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ProcTemp(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_ProcTemp.value, "Fault_ProcTemp", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Processor temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_ProcTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ProcTemp()
    {
        return getStickyFault_ProcTemp(true);
    }
    
    /**
     * Processor temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_ProcTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ProcTemp(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_ProcTemp.value, "StickyFault_ProcTemp", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Device temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_DeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_DeviceTemp()
    {
        return getFault_DeviceTemp(true);
    }
    
    /**
     * Device temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_DeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_DeviceTemp(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_DeviceTemp.value, "Fault_DeviceTemp", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Device temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_DeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_DeviceTemp()
    {
        return getStickyFault_DeviceTemp(true);
    }
    
    /**
     * Device temperature exceeded limit
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_DeviceTemp Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_DeviceTemp(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_DeviceTemp.value, "StickyFault_DeviceTemp", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Device supply voltage dropped to near brownout levels
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_Undervoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_Undervoltage()
    {
        return getFault_Undervoltage(true);
    }
    
    /**
     * Device supply voltage dropped to near brownout levels
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_Undervoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_Undervoltage(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_Undervoltage.value, "Fault_Undervoltage", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Device supply voltage dropped to near brownout levels
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_Undervoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_Undervoltage()
    {
        return getStickyFault_Undervoltage(true);
    }
    
    /**
     * Device supply voltage dropped to near brownout levels
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_Undervoltage Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_Undervoltage(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_Undervoltage.value, "StickyFault_Undervoltage", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Device boot while detecting the enable signal
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_BootDuringEnable Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_BootDuringEnable()
    {
        return getFault_BootDuringEnable(true);
    }
    
    /**
     * Device boot while detecting the enable signal
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_BootDuringEnable Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_BootDuringEnable(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_BootDuringEnable.value, "Fault_BootDuringEnable", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Device boot while detecting the enable signal
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_BootDuringEnable Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_BootDuringEnable()
    {
        return getStickyFault_BootDuringEnable(true);
    }
    
    /**
     * Device boot while detecting the enable signal
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_BootDuringEnable Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_BootDuringEnable(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_BootDuringEnable.value, "StickyFault_BootDuringEnable", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * An unlicensed feature is in use, device may not behave as expected.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_UnlicensedFeatureInUse Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_UnlicensedFeatureInUse()
    {
        return getFault_UnlicensedFeatureInUse(true);
    }
    
    /**
     * An unlicensed feature is in use, device may not behave as expected.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_UnlicensedFeatureInUse Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_UnlicensedFeatureInUse(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_UnlicensedFeatureInUse.value, "Fault_UnlicensedFeatureInUse", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * An unlicensed feature is in use, device may not behave as expected.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_UnlicensedFeatureInUse Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_UnlicensedFeatureInUse()
    {
        return getStickyFault_UnlicensedFeatureInUse(true);
    }
    
    /**
     * An unlicensed feature is in use, device may not behave as expected.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_UnlicensedFeatureInUse Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_UnlicensedFeatureInUse(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_UnlicensedFeatureInUse.value, "StickyFault_UnlicensedFeatureInUse", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Bridge was disabled most likely due to supply voltage dropping too
     * low.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_BridgeBrownout Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_BridgeBrownout()
    {
        return getFault_BridgeBrownout(true);
    }
    
    /**
     * Bridge was disabled most likely due to supply voltage dropping too
     * low.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_BridgeBrownout Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_BridgeBrownout(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_BridgeBrownout.value, "Fault_BridgeBrownout", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Bridge was disabled most likely due to supply voltage dropping too
     * low.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_BridgeBrownout Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_BridgeBrownout()
    {
        return getStickyFault_BridgeBrownout(true);
    }
    
    /**
     * Bridge was disabled most likely due to supply voltage dropping too
     * low.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_BridgeBrownout Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_BridgeBrownout(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_BridgeBrownout.value, "StickyFault_BridgeBrownout", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor has reset.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_RemoteSensorReset Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_RemoteSensorReset()
    {
        return getFault_RemoteSensorReset(true);
    }
    
    /**
     * The remote sensor has reset.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_RemoteSensorReset Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_RemoteSensorReset(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_RemoteSensorReset.value, "Fault_RemoteSensorReset", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor has reset.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_RemoteSensorReset Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_RemoteSensorReset()
    {
        return getStickyFault_RemoteSensorReset(true);
    }
    
    /**
     * The remote sensor has reset.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_RemoteSensorReset Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_RemoteSensorReset(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_RemoteSensorReset.value, "StickyFault_RemoteSensorReset", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote Talon used for differential control is not present on
     * CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_MissingDifferentialFX Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_MissingDifferentialFX()
    {
        return getFault_MissingDifferentialFX(true);
    }
    
    /**
     * The remote Talon used for differential control is not present on
     * CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_MissingDifferentialFX Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_MissingDifferentialFX(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_MissingDifferentialFX.value, "Fault_MissingDifferentialFX", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote Talon used for differential control is not present on
     * CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_MissingDifferentialFX Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_MissingDifferentialFX()
    {
        return getStickyFault_MissingDifferentialFX(true);
    }
    
    /**
     * The remote Talon used for differential control is not present on
     * CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_MissingDifferentialFX Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_MissingDifferentialFX(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_MissingDifferentialFX.value, "StickyFault_MissingDifferentialFX", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor position has overflowed. Because of the nature of
     * remote sensors, it is possible for the remote sensor position to
     * overflow beyond what is supported by the status signal frame.
     * However, this is rare and cannot occur over the course of an FRC
     * match under normal use.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_RemoteSensorPosOverflow Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_RemoteSensorPosOverflow()
    {
        return getFault_RemoteSensorPosOverflow(true);
    }
    
    /**
     * The remote sensor position has overflowed. Because of the nature of
     * remote sensors, it is possible for the remote sensor position to
     * overflow beyond what is supported by the status signal frame.
     * However, this is rare and cannot occur over the course of an FRC
     * match under normal use.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_RemoteSensorPosOverflow Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_RemoteSensorPosOverflow(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_RemoteSensorPosOverflow.value, "Fault_RemoteSensorPosOverflow", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor position has overflowed. Because of the nature of
     * remote sensors, it is possible for the remote sensor position to
     * overflow beyond what is supported by the status signal frame.
     * However, this is rare and cannot occur over the course of an FRC
     * match under normal use.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_RemoteSensorPosOverflow Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_RemoteSensorPosOverflow()
    {
        return getStickyFault_RemoteSensorPosOverflow(true);
    }
    
    /**
     * The remote sensor position has overflowed. Because of the nature of
     * remote sensors, it is possible for the remote sensor position to
     * overflow beyond what is supported by the status signal frame.
     * However, this is rare and cannot occur over the course of an FRC
     * match under normal use.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_RemoteSensorPosOverflow Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_RemoteSensorPosOverflow(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_RemoteSensorPosOverflow.value, "StickyFault_RemoteSensorPosOverflow", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Supply Voltage has exceeded the maximum voltage rating of device.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_OverSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_OverSupplyV()
    {
        return getFault_OverSupplyV(true);
    }
    
    /**
     * Supply Voltage has exceeded the maximum voltage rating of device.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_OverSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_OverSupplyV(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_OverSupplyV.value, "Fault_OverSupplyV", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Supply Voltage has exceeded the maximum voltage rating of device.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_OverSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_OverSupplyV()
    {
        return getStickyFault_OverSupplyV(true);
    }
    
    /**
     * Supply Voltage has exceeded the maximum voltage rating of device.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_OverSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_OverSupplyV(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_OverSupplyV.value, "StickyFault_OverSupplyV", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Supply Voltage is unstable.  Ensure you are using a battery and
     * current limited power supply.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_UnstableSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_UnstableSupplyV()
    {
        return getFault_UnstableSupplyV(true);
    }
    
    /**
     * Supply Voltage is unstable.  Ensure you are using a battery and
     * current limited power supply.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_UnstableSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_UnstableSupplyV(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_UnstableSupplyV.value, "Fault_UnstableSupplyV", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Supply Voltage is unstable.  Ensure you are using a battery and
     * current limited power supply.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_UnstableSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_UnstableSupplyV()
    {
        return getStickyFault_UnstableSupplyV(true);
    }
    
    /**
     * Supply Voltage is unstable.  Ensure you are using a battery and
     * current limited power supply.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_UnstableSupplyV Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_UnstableSupplyV(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_UnstableSupplyV.value, "StickyFault_UnstableSupplyV", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Reverse limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_ReverseHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ReverseHardLimit()
    {
        return getFault_ReverseHardLimit(true);
    }
    
    /**
     * Reverse limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_ReverseHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ReverseHardLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_ReverseHardLimit.value, "Fault_ReverseHardLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Reverse limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_ReverseHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ReverseHardLimit()
    {
        return getStickyFault_ReverseHardLimit(true);
    }
    
    /**
     * Reverse limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_ReverseHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ReverseHardLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_ReverseHardLimit.value, "StickyFault_ReverseHardLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Forward limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_ForwardHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ForwardHardLimit()
    {
        return getFault_ForwardHardLimit(true);
    }
    
    /**
     * Forward limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_ForwardHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ForwardHardLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_ForwardHardLimit.value, "Fault_ForwardHardLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Forward limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_ForwardHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ForwardHardLimit()
    {
        return getStickyFault_ForwardHardLimit(true);
    }
    
    /**
     * Forward limit switch has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_ForwardHardLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ForwardHardLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_ForwardHardLimit.value, "StickyFault_ForwardHardLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Reverse soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_ReverseSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ReverseSoftLimit()
    {
        return getFault_ReverseSoftLimit(true);
    }
    
    /**
     * Reverse soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_ReverseSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ReverseSoftLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_ReverseSoftLimit.value, "Fault_ReverseSoftLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Reverse soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_ReverseSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ReverseSoftLimit()
    {
        return getStickyFault_ReverseSoftLimit(true);
    }
    
    /**
     * Reverse soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_ReverseSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ReverseSoftLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_ReverseSoftLimit.value, "StickyFault_ReverseSoftLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Forward soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_ForwardSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ForwardSoftLimit()
    {
        return getFault_ForwardSoftLimit(true);
    }
    
    /**
     * Forward soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_ForwardSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_ForwardSoftLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_ForwardSoftLimit.value, "Fault_ForwardSoftLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Forward soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_ForwardSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ForwardSoftLimit()
    {
        return getStickyFault_ForwardSoftLimit(true);
    }
    
    /**
     * Forward soft limit has been asserted.  Output is set to neutral.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_ForwardSoftLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_ForwardSoftLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_ForwardSoftLimit.value, "StickyFault_ForwardSoftLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote soft limit device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_MissingSoftLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_MissingSoftLimitRemote()
    {
        return getFault_MissingSoftLimitRemote(true);
    }
    
    /**
     * The remote soft limit device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_MissingSoftLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_MissingSoftLimitRemote(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_MissingRemSoftLim.value, "Fault_MissingSoftLimitRemote", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote soft limit device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_MissingSoftLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_MissingSoftLimitRemote()
    {
        return getStickyFault_MissingSoftLimitRemote(true);
    }
    
    /**
     * The remote soft limit device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_MissingSoftLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_MissingSoftLimitRemote(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_MissingRemSoftLim.value, "StickyFault_MissingSoftLimitRemote", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote limit switch device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_MissingHardLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_MissingHardLimitRemote()
    {
        return getFault_MissingHardLimitRemote(true);
    }
    
    /**
     * The remote limit switch device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_MissingHardLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_MissingHardLimitRemote(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_MissingRemHardLim.value, "Fault_MissingHardLimitRemote", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote limit switch device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_MissingHardLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_MissingHardLimitRemote()
    {
        return getStickyFault_MissingHardLimitRemote(true);
    }
    
    /**
     * The remote limit switch device is not present on CAN Bus.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_MissingHardLimitRemote Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_MissingHardLimitRemote(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_MissingRemHardLim.value, "StickyFault_MissingHardLimitRemote", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor's data is no longer trusted. This can happen if
     * the remote sensor disappears from the CAN bus or if the remote
     * sensor indicates its data is no longer valid, such as when a
     * CANcoder's magnet strength falls into the "red" range.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_RemoteSensorDataInvalid Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_RemoteSensorDataInvalid()
    {
        return getFault_RemoteSensorDataInvalid(true);
    }
    
    /**
     * The remote sensor's data is no longer trusted. This can happen if
     * the remote sensor disappears from the CAN bus or if the remote
     * sensor indicates its data is no longer valid, such as when a
     * CANcoder's magnet strength falls into the "red" range.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_RemoteSensorDataInvalid Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_RemoteSensorDataInvalid(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_MissingRemoteSensor.value, "Fault_RemoteSensorDataInvalid", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor's data is no longer trusted. This can happen if
     * the remote sensor disappears from the CAN bus or if the remote
     * sensor indicates its data is no longer valid, such as when a
     * CANcoder's magnet strength falls into the "red" range.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_RemoteSensorDataInvalid Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_RemoteSensorDataInvalid()
    {
        return getStickyFault_RemoteSensorDataInvalid(true);
    }
    
    /**
     * The remote sensor's data is no longer trusted. This can happen if
     * the remote sensor disappears from the CAN bus or if the remote
     * sensor indicates its data is no longer valid, such as when a
     * CANcoder's magnet strength falls into the "red" range.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_RemoteSensorDataInvalid Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_RemoteSensorDataInvalid(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_MissingRemoteSensor.value, "StickyFault_RemoteSensorDataInvalid", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor used for fusion has fallen out of sync to the
     * local sensor. A re-synchronization has occurred, which may cause a
     * discontinuity. This typically happens if there is significant slop
     * in the mechanism, or if the RotorToSensorRatio configuration
     * parameter is incorrect.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_FusedSensorOutOfSync Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_FusedSensorOutOfSync()
    {
        return getFault_FusedSensorOutOfSync(true);
    }
    
    /**
     * The remote sensor used for fusion has fallen out of sync to the
     * local sensor. A re-synchronization has occurred, which may cause a
     * discontinuity. This typically happens if there is significant slop
     * in the mechanism, or if the RotorToSensorRatio configuration
     * parameter is incorrect.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_FusedSensorOutOfSync Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_FusedSensorOutOfSync(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_FusedSensorOutOfSync.value, "Fault_FusedSensorOutOfSync", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * The remote sensor used for fusion has fallen out of sync to the
     * local sensor. A re-synchronization has occurred, which may cause a
     * discontinuity. This typically happens if there is significant slop
     * in the mechanism, or if the RotorToSensorRatio configuration
     * parameter is incorrect.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_FusedSensorOutOfSync Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_FusedSensorOutOfSync()
    {
        return getStickyFault_FusedSensorOutOfSync(true);
    }
    
    /**
     * The remote sensor used for fusion has fallen out of sync to the
     * local sensor. A re-synchronization has occurred, which may cause a
     * discontinuity. This typically happens if there is significant slop
     * in the mechanism, or if the RotorToSensorRatio configuration
     * parameter is incorrect.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_FusedSensorOutOfSync Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_FusedSensorOutOfSync(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_FusedSensorOutOfSync.value, "StickyFault_FusedSensorOutOfSync", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Stator current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_StatorCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_StatorCurrLimit()
    {
        return getFault_StatorCurrLimit(true);
    }
    
    /**
     * Stator current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_StatorCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_StatorCurrLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_StatorCurrLimit.value, "Fault_StatorCurrLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Stator current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_StatorCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_StatorCurrLimit()
    {
        return getStickyFault_StatorCurrLimit(true);
    }
    
    /**
     * Stator current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_StatorCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_StatorCurrLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_StatorCurrLimit.value, "StickyFault_StatorCurrLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Supply current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_SupplyCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_SupplyCurrLimit()
    {
        return getFault_SupplyCurrLimit(true);
    }
    
    /**
     * Supply current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_SupplyCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_SupplyCurrLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_SupplyCurrLimit.value, "Fault_SupplyCurrLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Supply current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_SupplyCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_SupplyCurrLimit()
    {
        return getStickyFault_SupplyCurrLimit(true);
    }
    
    /**
     * Supply current limit occured.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_SupplyCurrLimit Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_SupplyCurrLimit(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_SupplyCurrLimit.value, "StickyFault_SupplyCurrLimit", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Using Fused CANcoder feature while unlicensed. Device has fallen
     * back to remote CANcoder.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_UsingFusedCANcoderWhileUnlicensed()
    {
        return getFault_UsingFusedCANcoderWhileUnlicensed(true);
    }
    
    /**
     * Using Fused CANcoder feature while unlicensed. Device has fallen
     * back to remote CANcoder.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_UsingFusedCANcoderWhileUnlicensed(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_UsingFusedCCWhileUnlicensed.value, "Fault_UsingFusedCANcoderWhileUnlicensed", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Using Fused CANcoder feature while unlicensed. Device has fallen
     * back to remote CANcoder.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_UsingFusedCANcoderWhileUnlicensed()
    {
        return getStickyFault_UsingFusedCANcoderWhileUnlicensed(true);
    }
    
    /**
     * Using Fused CANcoder feature while unlicensed. Device has fallen
     * back to remote CANcoder.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_UsingFusedCANcoderWhileUnlicensed(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_UsingFusedCCWhileUnlicensed.value, "StickyFault_UsingFusedCANcoderWhileUnlicensed", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Static brake was momentarily disabled due to excessive braking
     * current while disabled.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return Fault_StaticBrakeDisabled Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_StaticBrakeDisabled()
    {
        return getFault_StaticBrakeDisabled(true);
    }
    
    /**
     * Static brake was momentarily disabled due to excessive braking
     * current while disabled.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return Fault_StaticBrakeDisabled Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getFault_StaticBrakeDisabled(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.Fault_TALONFX_StaticBrakeDisabled.value, "Fault_StaticBrakeDisabled", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
        
    /**
     * Static brake was momentarily disabled due to excessive braking
     * current while disabled.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return StickyFault_StaticBrakeDisabled Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_StaticBrakeDisabled()
    {
        return getStickyFault_StaticBrakeDisabled(true);
    }
    
    /**
     * Static brake was momentarily disabled due to excessive braking
     * current while disabled.
     * 
     * <ul>
     *   <li> <b>Default Value:</b> False
     * </ul>
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN:</b> 4.0 Hz
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return StickyFault_StaticBrakeDisabled Status Signal Object
     */
    @Override
    public final StatusSignal<Boolean> getStickyFault_StaticBrakeDisabled(boolean refresh)
    {
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.StickyFault_TALONFX_StaticBrakeDisabled.value, "StickyFault_StaticBrakeDisabled", Boolean.class, val -> val != 0, true, refresh);
        return retval;
    }
    
    /**
     * Closed loop proportional component.
     * <p>
     * The portion of the closed loop output that is proportional to the
     * error. Alternatively, the kP contribution of the closed loop
     * output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopProportionalOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopProportionalOutput()
    {
        return getClosedLoopProportionalOutput(true);
    }
    
    /**
     * Closed loop proportional component.
     * <p>
     * The portion of the closed loop output that is proportional to the
     * error. Alternatively, the kP contribution of the closed loop
     * output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopProportionalOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopProportionalOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDOutput_ProportionalOutput_DC.value, "");
            toAdd.put(SpnValue.PRO_PIDOutput_ProportionalOutput_V.value, "");
            toAdd.put(SpnValue.PRO_PIDOutput_ProportionalOutput_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDOutput_ProportionalOutput_DC.value, "ClosedLoopProportionalOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Closed loop integrated component.
     * <p>
     * The portion of the closed loop output that is proportional to the
     * integrated error. Alternatively, the kI contribution of the closed
     * loop output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopIntegratedOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopIntegratedOutput()
    {
        return getClosedLoopIntegratedOutput(true);
    }
    
    /**
     * Closed loop integrated component.
     * <p>
     * The portion of the closed loop output that is proportional to the
     * integrated error. Alternatively, the kI contribution of the closed
     * loop output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopIntegratedOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopIntegratedOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDStateEnables_IntegratedAccum_DC.value, "");
            toAdd.put(SpnValue.PRO_PIDStateEnables_IntegratedAccum_V.value, "");
            toAdd.put(SpnValue.PRO_PIDStateEnables_IntegratedAccum_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDStateEnables_IntegratedAccum_DC.value, "ClosedLoopIntegratedOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Feedforward passed by the user.
     * <p>
     * This is the general feedforward that the user provides for the
     * closed loop.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopFeedForward Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopFeedForward()
    {
        return getClosedLoopFeedForward(true);
    }
    
    /**
     * Feedforward passed by the user.
     * <p>
     * This is the general feedforward that the user provides for the
     * closed loop.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopFeedForward Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopFeedForward(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDStateEnables_FeedForward_DC.value, "");
            toAdd.put(SpnValue.PRO_PIDStateEnables_FeedForward_V.value, "");
            toAdd.put(SpnValue.PRO_PIDStateEnables_FeedForward_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDStateEnables_FeedForward_DC.value, "ClosedLoopFeedForward", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Closed loop derivative component.
     * <p>
     * The portion of the closed loop output that is proportional to the
     * deriviative of error. Alternatively, the kD contribution of the
     * closed loop output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopDerivativeOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopDerivativeOutput()
    {
        return getClosedLoopDerivativeOutput(true);
    }
    
    /**
     * Closed loop derivative component.
     * <p>
     * The portion of the closed loop output that is proportional to the
     * deriviative of error. Alternatively, the kD contribution of the
     * closed loop output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopDerivativeOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopDerivativeOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDOutput_DerivativeOutput_DC.value, "");
            toAdd.put(SpnValue.PRO_PIDOutput_DerivativeOutput_V.value, "");
            toAdd.put(SpnValue.PRO_PIDOutput_DerivativeOutput_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDOutput_DerivativeOutput_DC.value, "ClosedLoopDerivativeOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Closed loop total output.
     * <p>
     * The total output of the closed loop output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopOutput()
    {
        return getClosedLoopOutput(true);
    }
    
    /**
     * Closed loop total output.
     * <p>
     * The total output of the closed loop output.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDOutput_Output_DC.value, "");
            toAdd.put(SpnValue.PRO_PIDOutput_Output_V.value, "");
            toAdd.put(SpnValue.PRO_PIDOutput_Output_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDOutput_Output_DC.value, "ClosedLoopOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Value that the closed loop is targeting.
     * <p>
     * This is the value that the closed loop PID controller targets.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopReference Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopReference()
    {
        return getClosedLoopReference(true);
    }
    
    /**
     * Value that the closed loop is targeting.
     * <p>
     * This is the value that the closed loop PID controller targets.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopReference Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopReference(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDRefPIDErr_PIDRef_Position.value, "");
            toAdd.put(SpnValue.PRO_PIDRefPIDErr_PIDRef_Velocity.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDRefPIDErr_PIDRef_Position.value, "ClosedLoopReference", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Derivative of the target that the closed loop is targeting.
     * <p>
     * This is the change in the closed loop reference. This may be used
     * in the feed-forward calculation, the derivative-error, or in
     * application of the signage for kS. Typically, this represents the
     * target velocity during Motion Magic®.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopReferenceSlope Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopReferenceSlope()
    {
        return getClosedLoopReferenceSlope(true);
    }
    
    /**
     * Derivative of the target that the closed loop is targeting.
     * <p>
     * This is the change in the closed loop reference. This may be used
     * in the feed-forward calculation, the derivative-error, or in
     * application of the signage for kS. Typically, this represents the
     * target velocity during Motion Magic®.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopReferenceSlope Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopReferenceSlope(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDRefSlopeECUTime_ReferenceSlope_Position.value, "");
            toAdd.put(SpnValue.PRO_PIDRefSlopeECUTime_ReferenceSlope_Velocity.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDRefSlopeECUTime_ReferenceSlope_Position.value, "ClosedLoopReferenceSlope", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * The difference between target reference and current measurement.
     * <p>
     * This is the value that is treated as the error in the PID loop.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return ClosedLoopError Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopError()
    {
        return getClosedLoopError(true);
    }
    
    /**
     * The difference between target reference and current measurement.
     * <p>
     * This is the value that is treated as the error in the PID loop.
     * <p>
     * When using differential control, this applies to the average axis.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return ClosedLoopError Status Signal object
     */
    @Override
    public final StatusSignal<Double> getClosedLoopError(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_PIDRefPIDErr_PIDErr_Position.value, "");
            toAdd.put(SpnValue.PRO_PIDRefPIDErr_PIDErr_Velocity.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_PIDRefPIDErr_PIDErr_Position.value, "ClosedLoopError", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * The calculated motor output for differential followers.
     * <p>
     * This is a torque request when using the TorqueCurrentFOC control
     * output type, a voltage request when using the Voltage control
     * output type, and a duty cycle when using the DutyCycle control
     * output type.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialOutput()
    {
        return getDifferentialOutput(true);
    }
    
    /**
     * The calculated motor output for differential followers.
     * <p>
     * This is a torque request when using the TorqueCurrentFOC control
     * output type, a voltage request when using the Voltage control
     * output type, and a duty cycle when using the DutyCycle control
     * output type.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_DutyCycle.value, "");
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_Voltage.value, "");
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_TorqueCurrent.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_PIDState_Diff_DutyCycle.value, "DifferentialOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Differential closed loop proportional component.
     * <p>
     * The portion of the differential closed loop output (on the
     * difference axis) that is proportional to the error. Alternatively,
     * the kP contribution of the closed loop output.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopProportionalOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopProportionalOutput()
    {
        return getDifferentialClosedLoopProportionalOutput(true);
    }
    
    /**
     * Differential closed loop proportional component.
     * <p>
     * The portion of the differential closed loop output (on the
     * difference axis) that is proportional to the error. Alternatively,
     * the kP contribution of the closed loop output.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopProportionalOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopProportionalOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_DiffPIDOutput_ProportionalOutput_DC.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDOutput_ProportionalOutput_V.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDOutput_ProportionalOutput_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPIDOutput_ProportionalOutput_DC.value, "DifferentialClosedLoopProportionalOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Differential closed loop integrated component.
     * <p>
     * The portion of the differential closed loop output (on the
     * difference axis) that is proportional to the integrated error.
     * Alternatively, the kI contribution of the closed loop output.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopIntegratedOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopIntegratedOutput()
    {
        return getDifferentialClosedLoopIntegratedOutput(true);
    }
    
    /**
     * Differential closed loop integrated component.
     * <p>
     * The portion of the differential closed loop output (on the
     * difference axis) that is proportional to the integrated error.
     * Alternatively, the kI contribution of the closed loop output.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopIntegratedOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopIntegratedOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_IntegratedAccum_DC.value, "");
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_IntegratedAccum_V.value, "");
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_IntegratedAccum_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_PIDState_Diff_IntegratedAccum_DC.value, "DifferentialClosedLoopIntegratedOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Differential Feedforward passed by the user.
     * <p>
     * This is the general feedforward that the user provides for the
     * differential closed loop (on the difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopFeedForward Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopFeedForward()
    {
        return getDifferentialClosedLoopFeedForward(true);
    }
    
    /**
     * Differential Feedforward passed by the user.
     * <p>
     * This is the general feedforward that the user provides for the
     * differential closed loop (on the difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 100.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopFeedForward Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopFeedForward(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_FeedForward_DC.value, "");
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_FeedForward_V.value, "");
            toAdd.put(SpnValue.PRO_MotorOutput_PIDState_Diff_FeedForward_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_MotorOutput_PIDState_Diff_FeedForward_DC.value, "DifferentialClosedLoopFeedForward", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Differential closed loop derivative component.
     * <p>
     * The portion of the differential closed loop output (on the
     * difference axis) that is proportional to the deriviative of error.
     * Alternatively, the kD contribution of the closed loop output.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopDerivativeOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopDerivativeOutput()
    {
        return getDifferentialClosedLoopDerivativeOutput(true);
    }
    
    /**
     * Differential closed loop derivative component.
     * <p>
     * The portion of the differential closed loop output (on the
     * difference axis) that is proportional to the deriviative of error.
     * Alternatively, the kD contribution of the closed loop output.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopDerivativeOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopDerivativeOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_DiffPIDOutput_DerivativeOutput_DC.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDOutput_DerivativeOutput_V.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDOutput_DerivativeOutput_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPIDOutput_DerivativeOutput_DC.value, "DifferentialClosedLoopDerivativeOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Differential closed loop total output.
     * <p>
     * The total output of the differential closed loop output (on the
     * difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopOutput()
    {
        return getDifferentialClosedLoopOutput(true);
    }
    
    /**
     * Differential closed loop total output.
     * <p>
     * The total output of the differential closed loop output (on the
     * difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopOutput Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopOutput(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_DiffPIDOutput_Output_DC.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDOutput_Output_V.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDOutput_Output_A.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPIDOutput_Output_DC.value, "DifferentialClosedLoopOutput", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Value that the differential closed loop is targeting.
     * <p>
     * This is the value that the differential closed loop PID controller
     * targets (on the difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopReference Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopReference()
    {
        return getDifferentialClosedLoopReference(true);
    }
    
    /**
     * Value that the differential closed loop is targeting.
     * <p>
     * This is the value that the differential closed loop PID controller
     * targets (on the difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopReference Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopReference(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_DiffPIDRefPIDErr_PIDRef_Position.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDRefPIDErr_PIDRef_Velocity.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPIDRefPIDErr_PIDRef_Position.value, "DifferentialClosedLoopReference", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * Derivative of the target that the differential closed loop is
     * targeting.
     * <p>
     * This is the change in the closed loop reference (on the difference
     * axis). This may be used in the feed-forward calculation, the
     * derivative-error, or in application of the signage for kS.
     * Typically, this represents the target velocity during Motion
     * Magic®.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopReferenceSlope Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopReferenceSlope()
    {
        return getDifferentialClosedLoopReferenceSlope(true);
    }
    
    /**
     * Derivative of the target that the differential closed loop is
     * targeting.
     * <p>
     * This is the change in the closed loop reference (on the difference
     * axis). This may be used in the feed-forward calculation, the
     * derivative-error, or in application of the signage for kS.
     * Typically, this represents the target velocity during Motion
     * Magic®.
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopReferenceSlope Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopReferenceSlope(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_DiffPIDRefSlopeECUTime_ReferenceSlope_Position.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDRefSlopeECUTime_ReferenceSlope_Velocity.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPIDRefSlopeECUTime_ReferenceSlope_Position.value, "DifferentialClosedLoopReferenceSlope", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }
    
    /**
     * The difference between target differential reference and current
     * measurement.
     * <p>
     * This is the value that is treated as the error in the differential
     * PID loop (on the difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @return DifferentialClosedLoopError Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopError()
    {
        return getDifferentialClosedLoopError(true);
    }
    
    /**
     * The difference between target differential reference and current
     * measurement.
     * <p>
     * This is the value that is treated as the error in the differential
     * PID loop (on the difference axis).
     * 
     * Default Rates:
     * <ul>
     *   <li> <b>CAN 2.0:</b> 4.0 Hz
     *   <li> <b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
     * </ul>
     * <p>
     * This refreshes and returns a cached StatusSignal object.
     * 
     * @param refresh Whether to refresh the StatusSignal before returning it;
     *                defaults to true
     * @return DifferentialClosedLoopError Status Signal object
     */
    @Override
    public final StatusSignal<Double> getDifferentialClosedLoopError(boolean refresh)
    {
        MapGenerator mapFiller = () -> {
            var toAdd = new HashMap<Integer, String>();
            toAdd.put(SpnValue.PRO_DiffPIDRefPIDErr_PIDErr_Position.value, "");
            toAdd.put(SpnValue.PRO_DiffPIDRefPIDErr_PIDErr_Velocity.value, "");
            return toAdd;
        };
    
        @SuppressWarnings("unchecked")
        var retval = super.lookupStatusSignal(SpnValue.PRO_DiffPIDRefPIDErr_PIDErr_Position.value, "DifferentialClosedLoopError", mapFiller, Double.class, val -> val, true, refresh);
        return retval;
    }

    
    /**
     * Request a specified motor duty cycle.
     * <p>
     * This control mode will output a proportion of the supplied voltage
     * which is supplied by the user.
     * <ul>
     *   <li> <b>DutyCycleOut Parameters:</b> 
     *   <ul>
     *     <li> <b>Output:</b> Proportion of supply voltage to apply in fractional
     *                         units between -1 and +1
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DutyCycleOut request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request a specified motor current (field oriented control).
     * <p>
     * This control request will drive the motor to the requested motor
     * (stator) current value.  This leverages field oriented control
     * (FOC), which means greater peak power than what is documented. 
     * This scales to torque based on Motor's kT constant.
     * <ul>
     *   <li> <b>TorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Output:</b> Amount of motor current in Amperes
     *     <li> <b>MaxAbsDutyCycle:</b> The maximum absolute motor output that can
     *                                  be applied, which effectively limits the
     *                                  velocity. For example, 0.50 means no more
     *                                  than 50% output in either direction.  This
     *                                  is useful for preventing the motor from
     *                                  spinning to its terminal velocity when there
     *                                  is no external torque applied unto the
     *                                  rotor.  Note this is absolute maximum, so
     *                                  the value should be between zero and one.
     *     <li> <b>Deadband:</b> Deadband in Amperes.  If torque request is within
     *                           deadband, the bridge output is neutral. If deadband
     *                           is set to zero then there is effectively no
     *                           deadband. Note if deadband is zero, a free spinning
     *                           motor will spin for quite a while as the firmware
     *                           attempts to hold the motor's bemf. If user expects
     *                           motor to cease spinning quickly with a demand of
     *                           zero, we recommend a deadband of one Ampere. This
     *                           value will be converted to an integral value of
     *                           amps.
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(TorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request a specified voltage.
     * <p>
     * This control mode will attempt to apply the specified voltage to
     * the motor. If the supply voltage is below the requested voltage,
     * the motor controller will output the supply voltage.
     * <ul>
     *   <li> <b>VoltageOut Parameters:</b> 
     *   <ul>
     *     <li> <b>Output:</b> Voltage to attempt to drive at
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(VoltageOut request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target position with duty cycle feedforward.
     * <p>
     * This control mode will set the motor's position setpoint to the
     * position specified by the user. In addition, it will apply an
     * additional duty cycle as an arbitrary feedforward value.
     * <ul>
     *   <li> <b>PositionDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
     *                           This is typically used for motion profiles
     *                           generated by the robot program.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in fractional units between
     *                              -1 and +1. This is added to the output of the
     *                              onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(PositionDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target position with voltage feedforward
     * <p>
     * This control mode will set the motor's position setpoint to the
     * position specified by the user. In addition, it will apply an
     * additional voltage as an arbitrary feedforward value.
     * <ul>
     *   <li> <b>PositionVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
     *                           This is typically used for motion profiles
     *                           generated by the robot program.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in volts. This is added to
     *                              the output of the onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(PositionVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target position with torque current feedforward.
     * <p>
     * This control mode will set the motor's position setpoint to the
     * position specified by the user. In addition, it will apply an
     * additional torque current as an arbitrary feedforward value.
     * <ul>
     *   <li> <b>PositionTorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
     *                           This is typically used for motion profiles
     *                           generated by the robot program.
     *     <li> <b>FeedForward:</b> Feedforward to apply in torque current in
     *                              Amperes. This is added to the output of the
     *                              onboard feedforward terms.
     *                              <p>
     *                              User can use motor's kT to scale Newton-meter to
     *                              Amperes.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(PositionTorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target velocity with duty cycle feedforward.
     * <p>
     * This control mode will set the motor's velocity setpoint to the
     * velocity specified by the user. In addition, it will apply an
     * additional voltage as an arbitrary feedforward value.
     * <ul>
     *   <li> <b>VelocityDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
     *     <li> <b>Acceleration:</b> Acceleration to drive toward in rotations per
     *                               second squared. This is typically used for
     *                               motion profiles generated by the robot program.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in fractional units between
     *                              -1 and +1. This is added to the output of the
     *                              onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(VelocityDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target velocity with voltage feedforward.
     * <p>
     * This control mode will set the motor's velocity setpoint to the
     * velocity specified by the user. In addition, it will apply an
     * additional voltage as an arbitrary feedforward value.
     * <ul>
     *   <li> <b>VelocityVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
     *     <li> <b>Acceleration:</b> Acceleration to drive toward in rotations per
     *                               second squared. This is typically used for
     *                               motion profiles generated by the robot program.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in volts This is added to
     *                              the output of the onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(VelocityVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target velocity with torque current feedforward.
     * <p>
     * This control mode will set the motor's velocity setpoint to the
     * velocity specified by the user. In addition, it will apply an
     * additional torque current as an arbitrary feedforward value.
     * <ul>
     *   <li> <b>VelocityTorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Velocity:</b> Velocity to drive toward in rotations per second.
     *     <li> <b>Acceleration:</b> Acceleration to drive toward in rotations per
     *                               second squared. This is typically used for
     *                               motion profiles generated by the robot program.
     *     <li> <b>FeedForward:</b> Feedforward to apply in torque current in
     *                              Amperes. This is added to the output of the
     *                              onboard feedforward terms.
     *                              <p>
     *                              User can use motor's kT to scale Newton-meter to
     *                              Amperes.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(VelocityTorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile.  Users can optionally provide a duty cycle feedforward.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and
     * (optional) Jerk specified via the Motion Magic® configuration
     * values.  This control mode does not use the Expo_kV or Expo_kA
     * configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>MotionMagicDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in fractional units between
     *                              -1 and +1. This is added to the output of the
     *                              onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile.  Users can optionally provide a voltage feedforward.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and
     * (optional) Jerk specified via the Motion Magic® configuration
     * values.  This control mode does not use the Expo_kV or Expo_kA
     * configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>MotionMagicVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in volts. This is added to
     *                              the output of the onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile.  Users can optionally provide a torque current
     * feedforward.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and
     * (optional) Jerk specified via the Motion Magic® configuration
     * values.  This control mode does not use the Expo_kV or Expo_kA
     * configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is based on
     * torque current, so relevant closed-loop gains will use Amperes for
     * the numerator.
     * <ul>
     *   <li> <b>MotionMagicTorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>FeedForward:</b> Feedforward to apply in torque current in
     *                              Amperes. This is added to the output of the
     *                              onboard feedforward terms.
     *                              <p>
     *                              User can use motor's kT to scale Newton-meter to
     *                              Amperes.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicTorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request a specified motor duty cycle with a differential position
     * closed-loop.
     * <p>
     * This control mode will output a proportion of the supplied voltage
     * which is supplied by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * <ul>
     *   <li> <b>DifferentialDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageOutput:</b> Proportion of supply voltage to apply on the
     *                                Average axis in fractional units between -1
     *                                and +1.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive towards
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request a specified voltage with a differential position
     * closed-loop.
     * <p>
     * This control mode will attempt to apply the specified voltage to
     * the motor. If the supply voltage is below the requested voltage,
     * the motor controller will output the supply voltage. It will also
     * set the motor's differential position setpoint to the specified
     * position.
     * <ul>
     *   <li> <b>DifferentialVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageOutput:</b> Voltage to attempt to drive at on the Average
     *                                axis.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive towards
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target position with a differential position
     * setpoint.
     * <p>
     * This control mode will set the motor's position setpoint to the
     * position specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * <ul>
     *   <li> <b>DifferentialPositionDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>AveragePosition:</b> Average position to drive toward in
     *                                  rotations.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialPositionDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target position with a differential position
     * setpoint
     * <p>
     * This control mode will set the motor's position setpoint to the
     * position specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * <ul>
     *   <li> <b>DifferentialPositionVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>AveragePosition:</b> Average position to drive toward in
     *                                  rotations.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialPositionVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target velocity with a differential position
     * setpoint.
     * <p>
     * This control mode will set the motor's velocity setpoint to the
     * velocity specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * <ul>
     *   <li> <b>DifferentialVelocityDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageVelocity:</b> Average velocity to drive toward in
     *                                  rotations per second.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialVelocityDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request PID to target velocity with a differential position
     * setpoint.
     * <p>
     * This control mode will set the motor's velocity setpoint to the
     * velocity specified by the user. It will also set the motor's
     * differential position setpoint to the specified position.
     * <ul>
     *   <li> <b>DifferentialVelocityVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageVelocity:</b> Average velocity to drive toward in
     *                                  rotations per second.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialVelocityVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile, and PID to a differential position setpoint.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and
     * (optional) Jerk specified via the Motion Magic® configuration
     * values.  This control mode does not use the Expo_kV or Expo_kA
     * configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>DifferentialMotionMagicDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>AveragePosition:</b> Average position to drive toward in
     *                                  rotations.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialMotionMagicDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile, and PID to a differential position setpoint.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity, Acceleration, and
     * (optional) Jerk specified via the Motion Magic® configuration
     * values.  This control mode does not use the Expo_kV or Expo_kA
     * configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>DifferentialMotionMagicVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>AveragePosition:</b> Average position to drive toward in
     *                                  rotations.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialMotionMagicVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using an
     * exponential motion profile, and PID to a differential position
     * setpoint.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Note that unlike the slot gains, the Expo_kV and Expo_kA
     * configs are always in output units of Volts.
     * <p>
     * Setting Cruise Velocity to 0 will allow the profile to run to the
     * max possible velocity based on Expo_kV.  This control mode does not
     * use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>DifferentialMotionMagicExpoDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>AveragePosition:</b> Average position to drive toward in
     *                                  rotations.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialMotionMagicExpoDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using an
     * exponential motion profile, and PID to a differential position
     * setpoint.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Note that unlike the slot gains, the Expo_kV and Expo_kA
     * configs are always in output units of Volts.
     * <p>
     * Setting Cruise Velocity to 0 will allow the profile to run to the
     * max possible velocity based on Expo_kV.  This control mode does not
     * use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>DifferentialMotionMagicExpoVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>AveragePosition:</b> Average position to drive toward in
     *                                  rotations.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialMotionMagicExpoVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final velocity using a motion
     * profile, and PID to a differential position setpoint.  This allows
     * smooth transitions between velocity set points.
     * <p>
     * Motion Magic® Velocity produces a motion profile in real-time while
     * attempting to honor the specified Acceleration and (optional) Jerk.
     *  This control mode does not use the CruiseVelocity, Expo_kV, or
     * Expo_kA configs.
     * <p>
     * Acceleration and jerk are specified in the Motion Magic® persistent
     * configuration values.  If Jerk is set to zero, Motion Magic® will
     * produce a trapezoidal acceleration profile.
     * <p>
     * Target velocity can also be changed on-the-fly and Motion Magic®
     * will do its best to adjust the profile.  This control mode is duty
     * cycle based, so relevant closed-loop gains will use fractional duty
     * cycle for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>DifferentialMotionMagicVelocityDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageVelocity:</b> Average velocity to drive toward in
     *                                  rotations per second.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialMotionMagicVelocityDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final velocity using a motion
     * profile, and PID to a differential position setpoint.  This allows
     * smooth transitions between velocity set points.
     * <p>
     * Motion Magic® Velocity produces a motion profile in real-time while
     * attempting to honor the specified Acceleration and (optional) Jerk.
     *  This control mode does not use the CruiseVelocity, Expo_kV, or
     * Expo_kA configs.
     * <p>
     * Acceleration and jerk are specified in the Motion Magic® persistent
     * configuration values.  If Jerk is set to zero, Motion Magic® will
     * produce a trapezoidal acceleration profile.
     * <p>
     * Target velocity can also be changed on-the-fly and Motion Magic®
     * will do its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>DifferentialMotionMagicVelocityVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageVelocity:</b> Average velocity to drive toward in
     *                                  rotations per second.
     *     <li> <b>DifferentialPosition:</b> Differential position to drive toward
     *                                       in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>AverageSlot:</b> Select which gains are applied to the average
     *                              controller by selecting the slot.  Use the
     *                              configuration api to set the gain values for the
     *                              selected slot before enabling this feature. Slot
     *                              must be within [0,2].
     *     <li> <b>DifferentialSlot:</b> Select which gains are applied to the
     *                                   differential controller by selecting the
     *                                   slot.  Use the configuration api to set the
     *                                   gain values for the selected slot before
     *                                   enabling this feature. Slot must be within
     *                                   [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialMotionMagicVelocityVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Follow the motor output of another Talon.
     * <p>
     * If Talon is in torque control, the torque is copied - which will
     * increase the total torque applied. If Talon is in duty cycle output
     * control, the duty cycle is matched. If Talon is in voltage output
     * control, the motor voltage is matched. Motor direction either
     * matches the leader's configured direction or opposes it based on
     * the MotorAlignment.
     * <p>
     * The leader must enable the status signal corresponding to its
     * control output type (DutyCycle, MotorVoltage, TorqueCurrent). The
     * update rate of the status signal determines the update rate of the
     * follower's output and should be no slower than 20 Hz.
     * <ul>
     *   <li> <b>Follower Parameters:</b> 
     *   <ul>
     *     <li> <b>LeaderID:</b> Device ID of the leader to follow.
     *     <li> <b>MotorAlignment:</b> Set to Aligned for motor invert to match the
     *                                 leader's configured Invert - which is typical
     *                                 when leader and follower are mechanically
     *                                 linked and spin in the same direction.  Set
     *                                 to Opposed for motor invert to oppose the
     *                                 leader's configured Invert - this is typical
     *                                 where the leader and follower mechanically
     *                                 spin in opposite directions.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Follower request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Follow the motor output of another Talon while ignoring the
     * leader's invert setting.
     * <p>
     * If Talon is in torque control, the torque is copied - which will
     * increase the total torque applied. If Talon is in duty cycle output
     * control, the duty cycle is matched. If Talon is in voltage output
     * control, the motor voltage is matched. Motor direction is strictly
     * determined by the configured invert and not the leader. If you want
     * motor direction to match or oppose the leader, use Follower
     * instead.
     * <p>
     * The leader must enable the status signal corresponding to its
     * control output type (DutyCycle, MotorVoltage, TorqueCurrent). The
     * update rate of the status signal determines the update rate of the
     * follower's output and should be no slower than 20 Hz.
     * <ul>
     *   <li> <b>StrictFollower Parameters:</b> 
     *   <ul>
     *     <li> <b>LeaderID:</b> Device ID of the leader to follow.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(StrictFollower request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Follow the differential motor output of another Talon.
     * <p>
     * If Talon is in torque control, the differential torque is copied -
     * which will increase the total torque applied. If Talon is in duty
     * cycle output control, the differential duty cycle is matched. If
     * Talon is in voltage output control, the differential motor voltage
     * is matched. Motor direction either matches leader's configured
     * direction or opposes it based on the MotorAlignment.
     * <p>
     * The leader must enable its DifferentialOutput status signal. The
     * update rate of the status signal determines the update rate of the
     * follower's output and should be no slower than 20 Hz.
     * <ul>
     *   <li> <b>DifferentialFollower Parameters:</b> 
     *   <ul>
     *     <li> <b>LeaderID:</b> Device ID of the differential leader to follow.
     *     <li> <b>MotorAlignment:</b> Set to Aligned for motor invert to match the
     *                                 leader's configured Invert - which is typical
     *                                 when leader and follower are mechanically
     *                                 linked and spin in the same direction.  Set
     *                                 to Opposed for motor invert to oppose the
     *                                 leader's configured Invert - this is typical
     *                                 where the leader and follower mechanically
     *                                 spin in opposite directions.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialFollower request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Follow the differential motor output of another Talon while
     * ignoring the leader's invert setting.
     * <p>
     * If Talon is in torque control, the differential torque is copied -
     * which will increase the total torque applied. If Talon is in duty
     * cycle output control, the differential duty cycle is matched. If
     * Talon is in voltage output control, the differential motor voltage
     * is matched. Motor direction is strictly determined by the
     * configured invert and not the leader. If you want motor direction
     * to match or oppose the leader, use DifferentialFollower instead.
     * <p>
     * The leader must enable its DifferentialOutput status signal. The
     * update rate of the status signal determines the update rate of the
     * follower's output and should be no slower than 20 Hz.
     * <ul>
     *   <li> <b>DifferentialStrictFollower Parameters:</b> 
     *   <ul>
     *     <li> <b>LeaderID:</b> Device ID of the differential leader to follow.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DifferentialStrictFollower request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request neutral output of actuator. The applied brake type is
     * determined by the NeutralMode configuration.
     * <ul>
     *   <li> <b>NeutralOut Parameters:</b> 
     *   <ul>
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(NeutralOut request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Request coast neutral output of actuator.  The bridge is disabled
     * and the rotor is allowed to coast.
     * <ul>
     *   <li> <b>CoastOut Parameters:</b> 
     *   <ul>
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(CoastOut request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Applies full neutral-brake by shorting motor leads together.
     * <ul>
     *   <li> <b>StaticBrake Parameters:</b> 
     *   <ul>
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(StaticBrake request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Plays a single tone at the user specified frequency.
     * <ul>
     *   <li> <b>MusicTone Parameters:</b> 
     *   <ul>
     *     <li> <b>AudioFrequency:</b> Sound frequency to play.  A value of zero
     *                                 will silence the device. The effective
     *                                 frequency range is 10-20000 Hz.  Any nonzero
     *                                 frequency less than 10 Hz will be capped to
     *                                 10 Hz.  Any frequency above 20 kHz will be
     *                                 capped to 20 kHz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MusicTone request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final velocity using a motion
     * profile.  This allows smooth transitions between velocity set
     * points.  Users can optionally provide a duty cycle feedforward.
     * <p>
     * Motion Magic® Velocity produces a motion profile in real-time while
     * attempting to honor the specified Acceleration and (optional) Jerk.
     *  This control mode does not use the CruiseVelocity, Expo_kV, or
     * Expo_kA configs.
     * <p>
     * If the specified acceleration is zero, the Acceleration under
     * Motion Magic® configuration parameter is used instead.  This allows
     * for runtime adjustment of acceleration for advanced users.  Jerk is
     * also specified in the Motion Magic® persistent configuration
     * values.  If Jerk is set to zero, Motion Magic® will produce a
     * trapezoidal acceleration profile.
     * <p>
     * Target velocity can also be changed on-the-fly and Motion Magic®
     * will do its best to adjust the profile.  This control mode is duty
     * cycle based, so relevant closed-loop gains will use fractional duty
     * cycle for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>MotionMagicVelocityDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>Velocity:</b> Target velocity to drive toward in rotations per
     *                           second.  This can be changed on-the fly.
     *     <li> <b>Acceleration:</b> This is the absolute Acceleration to use
     *                               generating the profile.  If this parameter is
     *                               zero, the Acceleration persistent configuration
     *                               parameter is used instead. Acceleration is in
     *                               rotations per second squared.  If nonzero, the
     *                               signage does not matter as the absolute value
     *                               is used.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in fractional units between
     *                              -1 and +1. This is added to the output of the
     *                              onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicVelocityDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final velocity using a motion
     * profile.  This allows smooth transitions between velocity set
     * points.  Users can optionally provide a torque feedforward.
     * <p>
     * Motion Magic® Velocity produces a motion profile in real-time while
     * attempting to honor the specified Acceleration and (optional) Jerk.
     *  This control mode does not use the CruiseVelocity, Expo_kV, or
     * Expo_kA configs.
     * <p>
     * If the specified acceleration is zero, the Acceleration under
     * Motion Magic® configuration parameter is used instead.  This allows
     * for runtime adjustment of acceleration for advanced users.  Jerk is
     * also specified in the Motion Magic® persistent configuration
     * values.  If Jerk is set to zero, Motion Magic® will produce a
     * trapezoidal acceleration profile.
     * <p>
     * Target velocity can also be changed on-the-fly and Motion Magic®
     * will do its best to adjust the profile.  This control mode is based
     * on torque current, so relevant closed-loop gains will use Amperes
     * for the numerator.
     * <ul>
     *   <li> <b>MotionMagicVelocityTorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Velocity:</b> Target velocity to drive toward in rotations per
     *                           second.  This can be changed on-the fly.
     *     <li> <b>Acceleration:</b> This is the absolute Acceleration to use
     *                               generating the profile.  If this parameter is
     *                               zero, the Acceleration persistent configuration
     *                               parameter is used instead. Acceleration is in
     *                               rotations per second squared.  If nonzero, the
     *                               signage does not matter as the absolute value
     *                               is used.
     *     <li> <b>FeedForward:</b> Feedforward to apply in torque current in
     *                              Amperes. This is added to the output of the
     *                              onboard feedforward terms.
     *                              <p>
     *                              User can use motor's kT to scale Newton-meter to
     *                              Amperes.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicVelocityTorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final velocity using a motion
     * profile.  This allows smooth transitions between velocity set
     * points.  Users can optionally provide a voltage feedforward.
     * <p>
     * Motion Magic® Velocity produces a motion profile in real-time while
     * attempting to honor the specified Acceleration and (optional) Jerk.
     *  This control mode does not use the CruiseVelocity, Expo_kV, or
     * Expo_kA configs.
     * <p>
     * If the specified acceleration is zero, the Acceleration under
     * Motion Magic® configuration parameter is used instead.  This allows
     * for runtime adjustment of acceleration for advanced users.  Jerk is
     * also specified in the Motion Magic® persistent configuration
     * values.  If Jerk is set to zero, Motion Magic® will produce a
     * trapezoidal acceleration profile.
     * <p>
     * Target velocity can also be changed on-the-fly and Motion Magic®
     * will do its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>MotionMagicVelocityVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>Velocity:</b> Target velocity to drive toward in rotations per
     *                           second.  This can be changed on-the fly.
     *     <li> <b>Acceleration:</b> This is the absolute Acceleration to use
     *                               generating the profile.  If this parameter is
     *                               zero, the Acceleration persistent configuration
     *                               parameter is used instead. Acceleration is in
     *                               rotations per second squared.  If nonzero, the
     *                               signage does not matter as the absolute value
     *                               is used.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in volts. This is added to
     *                              the output of the onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicVelocityVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a duty
     * cycle feedforward.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Note that unlike the slot gains, the Expo_kV and Expo_kA
     * configs are always in output units of Volts.
     * <p>
     * Setting Cruise Velocity to 0 will allow the profile to run to the
     * max possible velocity based on Expo_kV.  This control mode does not
     * use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>MotionMagicExpoDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in fractional units between
     *                              -1 and +1. This is added to the output of the
     *                              onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicExpoDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a voltage
     * feedforward.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Note that unlike the slot gains, the Expo_kV and Expo_kA
     * configs are always in output units of Volts.
     * <p>
     * Setting Cruise Velocity to 0 will allow the profile to run to the
     * max possible velocity based on Expo_kV.  This control mode does not
     * use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>MotionMagicExpoVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in volts. This is added to
     *                              the output of the onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicExpoVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using an
     * exponential motion profile.  Users can optionally provide a torque
     * current feedforward.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the Cruise Velocity (optional) and the
     * mechanism kV and kA, specified via the Motion Magic® configuration
     * values.  Note that unlike the slot gains, the Expo_kV and Expo_kA
     * configs are always in output units of Volts.
     * <p>
     * Setting Cruise Velocity to 0 will allow the profile to run to the
     * max possible velocity based on Expo_kV.  This control mode does not
     * use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is based on
     * torque current, so relevant closed-loop gains will use Amperes for
     * the numerator.
     * <ul>
     *   <li> <b>MotionMagicExpoTorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>FeedForward:</b> Feedforward to apply in torque current in
     *                              Amperes. This is added to the output of the
     *                              onboard feedforward terms.
     *                              <p>
     *                              User can use motor's kT to scale Newton-meter to
     *                              Amperes.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(MotionMagicExpoTorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile.  This dynamic request allows runtime changes to Cruise
     * Velocity, Acceleration, and (optional) Jerk.  Users can optionally
     * provide a duty cycle feedforward.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and (optional) Jerk.  This control mode does not use the Expo_kV or
     * Expo_kA configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile. This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>DynamicMotionMagicDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does
     *                           not matter as the device will use the absolute
     *                           value for profile generation.
     *     <li> <b>Acceleration:</b> Acceleration for profiling.  The signage does
     *                               not matter as the device will use the absolute
     *                               value for profile generation
     *     <li> <b>Jerk:</b> Jerk for profiling.  The signage does not matter as the
     *                       device will use the absolute value for profile
     *                       generation.
     *                       <p>
     *                       Jerk is optional; if this is set to zero, then Motion
     *                       Magic® will not apply a Jerk limit.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in fractional units between
     *                              -1 and +1. This is added to the output of the
     *                              onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DynamicMotionMagicDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile.  This dynamic request allows runtime changes to Cruise
     * Velocity, Acceleration, and (optional) Jerk.  Users can optionally
     * provide a voltage feedforward.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and (optional) Jerk.  This control mode does not use the Expo_kV or
     * Expo_kA configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>DynamicMotionMagicVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does
     *                           not matter as the device will use the absolute
     *                           value for profile generation.
     *     <li> <b>Acceleration:</b> Acceleration for profiling.  The signage does
     *                               not matter as the device will use the absolute
     *                               value for profile generation.
     *     <li> <b>Jerk:</b> Jerk for profiling.  The signage does not matter as the
     *                       device will use the absolute value for profile
     *                       generation.
     *                       <p>
     *                       Jerk is optional; if this is set to zero, then Motion
     *                       Magic® will not apply a Jerk limit.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in volts. This is added to
     *                              the output of the onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DynamicMotionMagicVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® to target a final position using a motion
     * profile.  This dynamic request allows runtime changes to Cruise
     * Velocity, Acceleration, and (optional) Jerk.  Users can optionally
     * provide a torque current feedforward.
     * <p>
     * Motion Magic® produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity, Acceleration,
     * and (optional) Jerk.  This control mode does not use the Expo_kV or
     * Expo_kA configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile. This control mode is based on
     * torque current, so relevant closed-loop gains will use Amperes for
     * the numerator.
     * <ul>
     *   <li> <b>DynamicMotionMagicTorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does
     *                           not matter as the device will use the absolute
     *                           value for profile generation.
     *     <li> <b>Acceleration:</b> Acceleration for profiling.  The signage does
     *                               not matter as the device will use the absolute
     *                               value for profile generation.
     *     <li> <b>Jerk:</b> Jerk for profiling.  The signage does not matter as the
     *                       device will use the absolute value for profile
     *                       generation.
     *                       <p>
     *                       Jerk is optional; if this is set to zero, then Motion
     *                       Magic® will not apply a Jerk limit.
     *     <li> <b>FeedForward:</b> Feedforward to apply in torque current in
     *                              Amperes. This is added to the output of the
     *                              onboard feedforward terms.
     *                              <p>
     *                              User can use motor's kT to scale Newton-meter to
     *                              Amperes.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DynamicMotionMagicTorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® Expo to target a final position using an
     * exponential motion profile.  This dynamic request allows runtime
     * changes to the profile kV, kA, and (optional) Cruise Velocity. 
     * Users can optionally provide a duty cycle feedforward.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity (optional) and
     * the mechanism kV and kA.  Note that unlike the slot gains, the
     * Expo_kV and Expo_kA parameters are always in output units of Volts.
     * <p>
     * Setting the Cruise Velocity to 0 will allow the profile to run to
     * the max possible velocity based on Expo_kV.  This control mode does
     * not use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile. This control mode is duty cycle
     * based, so relevant closed-loop gains will use fractional duty cycle
     * for the numerator:  +1.0 represents full forward output.
     * <ul>
     *   <li> <b>DynamicMotionMagicExpoDutyCycle Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>kV:</b> Mechanism kV for profiling.  Unlike the kV slot gain,
     *                     this is always in units of V/rps.
     *                     <p>
     *                     This represents the amount of voltage necessary to hold a
     *                     velocity.  In terms of the Motion Magic® Expo profile, a
     *                     higher kV results in a slower maximum velocity.
     *     <li> <b>kA:</b> Mechanism kA for profiling.  Unlike the kA slot gain,
     *                     this is always in units of V/rps².
     *                     <p>
     *                     This represents the amount of voltage necessary to
     *                     achieve an acceleration.  In terms of the Motion Magic®
     *                     Expo profile, a higher kA results in a slower
     *                     acceleration.
     *     <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does
     *                           not matter as the device will use the absolute
     *                           value for profile generation.  Setting this to 0
     *                           will allow the profile to run to the max possible
     *                           velocity based on Expo_kV.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in fractional units between
     *                              -1 and +1. This is added to the output of the
     *                              onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DynamicMotionMagicExpoDutyCycle request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® Expo to target a final position using an
     * exponential motion profile.  This dynamic request allows runtime
     * changes to the profile kV, kA, and (optional) Cruise Velocity. 
     * Users can optionally provide a voltage feedforward.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity (optional) and
     * the mechanism kV and kA.  Note that unlike the slot gains, the
     * Expo_kV and Expo_kA parameters are always in output units of Volts.
     * <p>
     * Setting the Cruise Velocity to 0 will allow the profile to run to
     * the max possible velocity based on Expo_kV.  This control mode does
     * not use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile.  This control mode is
     * voltage-based, so relevant closed-loop gains will use Volts for the
     * numerator.
     * <ul>
     *   <li> <b>DynamicMotionMagicExpoVoltage Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>kV:</b> Mechanism kV for profiling.  Unlike the kV slot gain,
     *                     this is always in units of V/rps.
     *                     <p>
     *                     This represents the amount of voltage necessary to hold a
     *                     velocity.  In terms of the Motion Magic® Expo profile, a
     *                     higher kV results in a slower maximum velocity.
     *     <li> <b>kA:</b> Mechanism kA for profiling.  Unlike the kA slot gain,
     *                     this is always in units of V/rps².
     *                     <p>
     *                     This represents the amount of voltage necessary to
     *                     achieve an acceleration.  In terms of the Motion Magic®
     *                     Expo profile, a higher kA results in a slower
     *                     acceleration.
     *     <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does
     *                           not matter as the device will use the absolute
     *                           value for profile generation.  Setting this to 0
     *                           will allow the profile to run to the max possible
     *                           velocity based on Expo_kV.
     *     <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires
     *                            Phoenix Pro), which increases peak power by ~15%
     *                            on supported devices (see {@link SupportsFOC}).
     *                            Set to false to use trapezoidal commutation.
     *                            <p>
     *                            FOC improves motor performance by leveraging
     *                            torque (current) control.  However, this may be
     *                            inconvenient for applications that require
     *                            specifying duty cycle or voltage.  CTR-Electronics
     *                            has developed a hybrid method that combines the
     *                            performances gains of FOC while still allowing
     *                            applications to provide duty cycle or voltage
     *                            demand.  This not to be confused with simple
     *                            sinusoidal control or phase voltage control which
     *                            lacks the performance gains.
     *     <li> <b>FeedForward:</b> Feedforward to apply in volts. This is added to
     *                              the output of the onboard feedforward terms.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the
     *                                          rotor when output is zero (or within
     *                                          deadband).  Set to false to use the
     *                                          NeutralMode configuration setting
     *                                          (default). This flag exists to
     *                                          provide the fundamental behavior of
     *                                          this control when output is zero,
     *                                          which is to provide 0V to the motor.
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DynamicMotionMagicExpoVoltage request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Requests Motion Magic® Expo to target a final position using an
     * exponential motion profile.  This dynamic request allows runtime
     * changes to the profile kV, kA, and (optional) Cruise Velocity. 
     * Users can optionally provide a torque current feedforward.
     * <p>
     * Motion Magic® Expo produces a motion profile in real-time while
     * attempting to honor the specified Cruise Velocity (optional) and
     * the mechanism kV and kA.  Note that unlike the slot gains, the
     * Expo_kV and Expo_kA parameters are always in output units of Volts.
     * <p>
     * Setting the Cruise Velocity to 0 will allow the profile to run to
     * the max possible velocity based on Expo_kV.  This control mode does
     * not use the Acceleration or Jerk configs.
     * <p>
     * Target position can be changed on-the-fly and Motion Magic® will do
     * its best to adjust the profile. This control mode is based on
     * torque current, so relevant closed-loop gains will use Amperes for
     * the numerator.
     * <ul>
     *   <li> <b>DynamicMotionMagicExpoTorqueCurrentFOC Parameters:</b> 
     *   <ul>
     *     <li> <b>Position:</b> Position to drive toward in rotations.
     *     <li> <b>kV:</b> Mechanism kV for profiling.  Unlike the kV slot gain,
     *                     this is always in units of V/rps.
     *                     <p>
     *                     This represents the amount of voltage necessary to hold a
     *                     velocity.  In terms of the Motion Magic® Expo profile, a
     *                     higher kV results in a slower maximum velocity.
     *     <li> <b>kA:</b> Mechanism kA for profiling.  Unlike the kA slot gain,
     *                     this is always in units of V/rps².
     *                     <p>
     *                     This represents the amount of voltage necessary to
     *                     achieve an acceleration.  In terms of the Motion Magic®
     *                     Expo profile, a higher kA results in a slower
     *                     acceleration.
     *     <li> <b>Velocity:</b> Cruise velocity for profiling.  The signage does
     *                           not matter as the device will use the absolute
     *                           value for profile generation.  Setting this to 0
     *                           will allow the profile to run to the max possible
     *                           velocity based on Expo_kV.
     *     <li> <b>FeedForward:</b> Feedforward to apply in torque current in
     *                              Amperes. This is added to the output of the
     *                              onboard feedforward terms.
     *                              <p>
     *                              User can use motor's kT to scale Newton-meter to
     *                              Amperes.
     *     <li> <b>Slot:</b> Select which gains are applied by selecting the slot. 
     *                       Use the configuration api to set the gain values for
     *                       the selected slot before enabling this feature. Slot
     *                       must be within [0,2].
     *     <li> <b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when
     *                                          output is zero (or within deadband).
     *                                           Set to false to use the NeutralMode
     *                                          configuration setting (default).
     *                                          This flag exists to provide the
     *                                          fundamental behavior of this control
     *                                          when output is zero, which is to
     *                                          provide 0A (zero torque).
     *     <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting. 
     *                                     This allows users to use other limit
     *                                     switch sensors connected to robot
     *                                     controller.  This also allows use of
     *                                     active sensors that require external
     *                                     power.
     *     <li> <b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit
     *                                       switches and the LimitForwardMotion and
     *                                       LimitReverseMotion parameters, instead
     *                                       allowing motion.
     *                                       <p>
     *                                       This can be useful on mechanisms such
     *                                       as an intake/feeder, where a limit
     *                                       switch stops motion while intaking but
     *                                       should be ignored when feeding to a
     *                                       shooter.
     *                                       <p>
     *                                       The hardware limit faults and
     *                                       Forward/ReverseLimit signals will still
     *                                       report the values of the limit switches
     *                                       regardless of this parameter.
     *     <li> <b>IgnoreSoftwareLimits:</b> Set to true to ignore software limits,
     *                                       instead allowing motion.
     *                                       <p>
     *                                       This can be useful when calibrating the
     *                                       zero point of a mechanism such as an
     *                                       elevator.
     *                                       <p>
     *                                       The software limit faults will still
     *                                       report the values of the software
     *                                       limits regardless of this parameter.
     *     <li> <b>UseTimesync:</b> Set to true to delay applying this control
     *                              request until a timesync boundary (requires
     *                              Phoenix Pro and CANivore). This eliminates the
     *                              impact of nondeterministic network delays in
     *                              exchange for a larger but deterministic control
     *                              latency.
     *                              <p>
     *                              This requires setting the ControlTimesyncFreqHz
     *                              config in MotorOutputConfigs. Additionally, when
     *                              this is enabled, the UpdateFreqHz of this
     *                              request should be set to 0 Hz.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(DynamicMotionMagicExpoTorqueCurrentFOC request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with duty cycle average target and position
     * difference target.
     * <ul>
     *   <li> <b>Diff_DutyCycleOut_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average DutyCycleOut request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_DutyCycleOut_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and position
     * difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_PositionDutyCycle_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionDutyCycle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionDutyCycle_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and position
     * difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_VelocityDutyCycle_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityDutyCYcle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityDutyCycle_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and position
     * difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_MotionMagicDutyCycle_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicDutyCycle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicDutyCycle_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * position difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoDutyCycle_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoDutyCycle request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoDutyCycle_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * position difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityDutyCycle_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityDutyCycle request
     *                                 of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityDutyCycle_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with duty cycle average target and velocity
     * difference target.
     * <ul>
     *   <li> <b>Diff_DutyCycleOut_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average DutyCycleOut request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_DutyCycleOut_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and velocity
     * difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_PositionDutyCycle_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionDutyCycle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionDutyCycle_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and velocity
     * difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_VelocityDutyCycle_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityDutyCycle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityDutyCycle_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and velocity
     * difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_MotionMagicDutyCycle_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicDutyCycle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicDutyCycle_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * velocity difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoDutyCycle_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoDutyCycle request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoDutyCycle_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * velocity difference target using duty cycle control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityDutyCycle_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityDutyCycle request
     *                                 of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityDutyCycle request
     *                                      of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityDutyCycle_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with duty cycle average target and duty cycle
     * difference target.
     * <ul>
     *   <li> <b>Diff_DutyCycleOut_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average DutyCycleOut request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential DutyCycleOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_DutyCycleOut_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and duty cycle
     * difference target.
     * <ul>
     *   <li> <b>Diff_PositionDutyCycle_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionDutyCycle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential DutyCycleOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionDutyCycle_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and duty cycle
     * difference target.
     * <ul>
     *   <li> <b>Diff_VelocityDutyCycle_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityDutyCYcle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential DutyCycleOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityDutyCycle_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and duty
     * cycle difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicDutyCycle_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicDutyCycle request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential DutyCycleOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicDutyCycle_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * duty cycle difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoDutyCycle_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoDutyCycle request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential DutyCycleOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoDutyCycle_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * duty cycle difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityDutyCycle_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityDutyCycle request
     *                                 of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential DutyCycleOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityDutyCycle_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with voltage average target and position
     * difference target.
     * <ul>
     *   <li> <b>Diff_VoltageOut_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VoltageOut request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VoltageOut_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and position
     * difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_PositionVoltage_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionVoltage_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and position
     * difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_VelocityVoltage_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityVoltage_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and position
     * difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVoltage_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVoltage_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * position difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoVoltage_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoVoltage_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * position difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityVoltage_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityVoltage request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityVoltage_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with voltage average target and velocity
     * difference target.
     * <ul>
     *   <li> <b>Diff_VoltageOut_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VoltageOut request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VoltageOut_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and velocity
     * difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_PositionVoltage_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionVoltage_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and velocity
     * difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_VelocityVoltage_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityVoltage_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and velocity
     * difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVoltage_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVoltage_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * velocity difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoVoltage_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoVoltage_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * velocity difference target using voltage control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityVoltage_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityVoltage request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityVoltage request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityVoltage_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with voltage average target and voltage
     * difference target.
     * <ul>
     *   <li> <b>Diff_VoltageOut_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VoltageOut request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VoltageOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VoltageOut_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and voltage
     * difference target.
     * <ul>
     *   <li> <b>Diff_PositionVoltage_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VoltageOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionVoltage_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and voltage
     * difference target.
     * <ul>
     *   <li> <b>Diff_VelocityVoltage_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VoltageOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityVoltage_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and voltage
     * difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicVoltage_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VoltageOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVoltage_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * voltage difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoVoltage_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoVoltage request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VoltageOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoVoltage_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * voltage difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityVoltage_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityVoltage request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VoltageOut request of the
     *                                      mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityVoltage_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with torque current average target and
     * position difference target.
     * <ul>
     *   <li> <b>Diff_TorqueCurrentFOC_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average TorqueCurrentFOC request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_TorqueCurrentFOC_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and position
     * difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_PositionTorqueCurrentFOC_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionTorqueCurrentFOC request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionTorqueCurrentFOC_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and position
     * difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_VelocityTorqueCurrentFOC_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityTorqueCurrentFOC request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityTorqueCurrentFOC_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and position
     * difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_MotionMagicTorqueCurrentFOC_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicTorqueCurrentFOC request
     *                                 of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicTorqueCurrentFOC_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * position difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoTorqueCurrentFOC_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoTorqueCurrentFOC
     *                                 request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoTorqueCurrentFOC_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * position difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityTorqueCurrentFOC_Position Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityTorqueCurrentFOC
     *                                 request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityTorqueCurrentFOC_Position request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with torque current average target and
     * velocity difference target.
     * <ul>
     *   <li> <b>Diff_TorqueCurrentFOC_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average TorqueCurrentFOC request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_TorqueCurrentFOC_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and velocity
     * difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_PositionTorqueCurrentFOC_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionTorqueCurrentFOC request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionTorqueCurrentFOC_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and velocity
     * difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_VelocityTorqueCurrentFOC_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityTorqueCurrentFOC request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityTorqueCurrentFOC_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and velocity
     * difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_MotionMagicTorqueCurrentFOC_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicTorqueCurrentFOC request
     *                                 of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicTorqueCurrentFOC_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * velocity difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoTorqueCurrentFOC_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoTorqueCurrentFOC
     *                                 request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoTorqueCurrentFOC_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * velocity difference target using torque current control.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityTorqueCurrentFOC_Velocity Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityTorqueCurrentFOC
     *                                 request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC
     *                                      request of the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityTorqueCurrentFOC_Velocity request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with torque current average target and torque
     * current difference target.
     * <ul>
     *   <li> <b>Diff_TorqueCurrentFOC_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average TorqueCurrentFOC request of the
     *                                 mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential TorqueCurrentFOC request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_TorqueCurrentFOC_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with position average target and torque
     * current difference target.
     * <ul>
     *   <li> <b>Diff_PositionTorqueCurrentFOC_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average PositionTorqueCurrentFOC request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential TorqueCurrentFOC request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_PositionTorqueCurrentFOC_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with velocity average target and torque
     * current difference target.
     * <ul>
     *   <li> <b>Diff_VelocityTorqueCurrentFOC_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average VelocityTorqueCurrentFOC request of
     *                                 the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential TorqueCurrentFOC request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_VelocityTorqueCurrentFOC_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® average target and torque
     * current difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicTorqueCurrentFOC_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicTorqueCurrentFOC request
     *                                 of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential TorqueCurrentFOC request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicTorqueCurrentFOC_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Expo average target and
     * torque current difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicExpoTorqueCurrentFOC_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicExpoTorqueCurrentFOC
     *                                 request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential TorqueCurrentFOC request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicExpoTorqueCurrentFOC_Open request)
    {
        return setControlPrivate(request);
    }
    
    /**
     * Differential control with Motion Magic® Velocity average target and
     * torque current difference target.
     * <ul>
     *   <li> <b>Diff_MotionMagicVelocityTorqueCurrentFOC_Open Parameters:</b> 
     *   <ul>
     *     <li> <b>AverageRequest:</b> Average MotionMagicVelocityTorqueCurrentFOC
     *                                 request of the mechanism.
     *     <li> <b>DifferentialRequest:</b> Differential TorqueCurrentFOC request of
     *                                      the mechanism.
     *   </ul>
     * </ul>
     *
     * @param request Control object to request of the device
     * @return Code response of the request
     */
    @Override
    public final StatusCode setControl(Diff_MotionMagicVelocityTorqueCurrentFOC_Open request)
    {
        return setControlPrivate(request);
    }

    /**
     * Control device with generic control request object.
     * <p>
     * User must make sure the specified object is castable to a valid control request,
     * otherwise this function will fail at run-time and return the NotSupported StatusCode
     *
     * @param request Control object to request of the device
     * @return Status Code of the request, 0 is OK
     */
    @Override
    public final StatusCode setControl(ControlRequest request)
    {
        if (request instanceof DutyCycleOut reqDutyCycleOut)
            return setControl(reqDutyCycleOut);
        if (request instanceof TorqueCurrentFOC reqTorqueCurrentFOC)
            return setControl(reqTorqueCurrentFOC);
        if (request instanceof VoltageOut reqVoltageOut)
            return setControl(reqVoltageOut);
        if (request instanceof PositionDutyCycle reqPositionDutyCycle)
            return setControl(reqPositionDutyCycle);
        if (request instanceof PositionVoltage reqPositionVoltage)
            return setControl(reqPositionVoltage);
        if (request instanceof PositionTorqueCurrentFOC reqPositionTorqueCurrentFOC)
            return setControl(reqPositionTorqueCurrentFOC);
        if (request instanceof VelocityDutyCycle reqVelocityDutyCycle)
            return setControl(reqVelocityDutyCycle);
        if (request instanceof VelocityVoltage reqVelocityVoltage)
            return setControl(reqVelocityVoltage);
        if (request instanceof VelocityTorqueCurrentFOC reqVelocityTorqueCurrentFOC)
            return setControl(reqVelocityTorqueCurrentFOC);
        if (request instanceof MotionMagicDutyCycle reqMotionMagicDutyCycle)
            return setControl(reqMotionMagicDutyCycle);
        if (request instanceof MotionMagicVoltage reqMotionMagicVoltage)
            return setControl(reqMotionMagicVoltage);
        if (request instanceof MotionMagicTorqueCurrentFOC reqMotionMagicTorqueCurrentFOC)
            return setControl(reqMotionMagicTorqueCurrentFOC);
        if (request instanceof DifferentialDutyCycle reqDifferentialDutyCycle)
            return setControl(reqDifferentialDutyCycle);
        if (request instanceof DifferentialVoltage reqDifferentialVoltage)
            return setControl(reqDifferentialVoltage);
        if (request instanceof DifferentialPositionDutyCycle reqDifferentialPositionDutyCycle)
            return setControl(reqDifferentialPositionDutyCycle);
        if (request instanceof DifferentialPositionVoltage reqDifferentialPositionVoltage)
            return setControl(reqDifferentialPositionVoltage);
        if (request instanceof DifferentialVelocityDutyCycle reqDifferentialVelocityDutyCycle)
            return setControl(reqDifferentialVelocityDutyCycle);
        if (request instanceof DifferentialVelocityVoltage reqDifferentialVelocityVoltage)
            return setControl(reqDifferentialVelocityVoltage);
        if (request instanceof DifferentialMotionMagicDutyCycle reqDifferentialMotionMagicDutyCycle)
            return setControl(reqDifferentialMotionMagicDutyCycle);
        if (request instanceof DifferentialMotionMagicVoltage reqDifferentialMotionMagicVoltage)
            return setControl(reqDifferentialMotionMagicVoltage);
        if (request instanceof DifferentialMotionMagicExpoDutyCycle reqDifferentialMotionMagicExpoDutyCycle)
            return setControl(reqDifferentialMotionMagicExpoDutyCycle);
        if (request instanceof DifferentialMotionMagicExpoVoltage reqDifferentialMotionMagicExpoVoltage)
            return setControl(reqDifferentialMotionMagicExpoVoltage);
        if (request instanceof DifferentialMotionMagicVelocityDutyCycle reqDifferentialMotionMagicVelocityDutyCycle)
            return setControl(reqDifferentialMotionMagicVelocityDutyCycle);
        if (request instanceof DifferentialMotionMagicVelocityVoltage reqDifferentialMotionMagicVelocityVoltage)
            return setControl(reqDifferentialMotionMagicVelocityVoltage);
        if (request instanceof Follower reqFollower)
            return setControl(reqFollower);
        if (request instanceof StrictFollower reqStrictFollower)
            return setControl(reqStrictFollower);
        if (request instanceof DifferentialFollower reqDifferentialFollower)
            return setControl(reqDifferentialFollower);
        if (request instanceof DifferentialStrictFollower reqDifferentialStrictFollower)
            return setControl(reqDifferentialStrictFollower);
        if (request instanceof NeutralOut reqNeutralOut)
            return setControl(reqNeutralOut);
        if (request instanceof CoastOut reqCoastOut)
            return setControl(reqCoastOut);
        if (request instanceof StaticBrake reqStaticBrake)
            return setControl(reqStaticBrake);
        if (request instanceof MusicTone reqMusicTone)
            return setControl(reqMusicTone);
        if (request instanceof MotionMagicVelocityDutyCycle reqMotionMagicVelocityDutyCycle)
            return setControl(reqMotionMagicVelocityDutyCycle);
        if (request instanceof MotionMagicVelocityTorqueCurrentFOC reqMotionMagicVelocityTorqueCurrentFOC)
            return setControl(reqMotionMagicVelocityTorqueCurrentFOC);
        if (request instanceof MotionMagicVelocityVoltage reqMotionMagicVelocityVoltage)
            return setControl(reqMotionMagicVelocityVoltage);
        if (request instanceof MotionMagicExpoDutyCycle reqMotionMagicExpoDutyCycle)
            return setControl(reqMotionMagicExpoDutyCycle);
        if (request instanceof MotionMagicExpoVoltage reqMotionMagicExpoVoltage)
            return setControl(reqMotionMagicExpoVoltage);
        if (request instanceof MotionMagicExpoTorqueCurrentFOC reqMotionMagicExpoTorqueCurrentFOC)
            return setControl(reqMotionMagicExpoTorqueCurrentFOC);
        if (request instanceof DynamicMotionMagicDutyCycle reqDynamicMotionMagicDutyCycle)
            return setControl(reqDynamicMotionMagicDutyCycle);
        if (request instanceof DynamicMotionMagicVoltage reqDynamicMotionMagicVoltage)
            return setControl(reqDynamicMotionMagicVoltage);
        if (request instanceof DynamicMotionMagicTorqueCurrentFOC reqDynamicMotionMagicTorqueCurrentFOC)
            return setControl(reqDynamicMotionMagicTorqueCurrentFOC);
        if (request instanceof DynamicMotionMagicExpoDutyCycle reqDynamicMotionMagicExpoDutyCycle)
            return setControl(reqDynamicMotionMagicExpoDutyCycle);
        if (request instanceof DynamicMotionMagicExpoVoltage reqDynamicMotionMagicExpoVoltage)
            return setControl(reqDynamicMotionMagicExpoVoltage);
        if (request instanceof DynamicMotionMagicExpoTorqueCurrentFOC reqDynamicMotionMagicExpoTorqueCurrentFOC)
            return setControl(reqDynamicMotionMagicExpoTorqueCurrentFOC);
        if (request instanceof Diff_DutyCycleOut_Position reqDiff_DutyCycleOut_Position)
            return setControl(reqDiff_DutyCycleOut_Position);
        if (request instanceof Diff_PositionDutyCycle_Position reqDiff_PositionDutyCycle_Position)
            return setControl(reqDiff_PositionDutyCycle_Position);
        if (request instanceof Diff_VelocityDutyCycle_Position reqDiff_VelocityDutyCycle_Position)
            return setControl(reqDiff_VelocityDutyCycle_Position);
        if (request instanceof Diff_MotionMagicDutyCycle_Position reqDiff_MotionMagicDutyCycle_Position)
            return setControl(reqDiff_MotionMagicDutyCycle_Position);
        if (request instanceof Diff_MotionMagicExpoDutyCycle_Position reqDiff_MotionMagicExpoDutyCycle_Position)
            return setControl(reqDiff_MotionMagicExpoDutyCycle_Position);
        if (request instanceof Diff_MotionMagicVelocityDutyCycle_Position reqDiff_MotionMagicVelocityDutyCycle_Position)
            return setControl(reqDiff_MotionMagicVelocityDutyCycle_Position);
        if (request instanceof Diff_DutyCycleOut_Velocity reqDiff_DutyCycleOut_Velocity)
            return setControl(reqDiff_DutyCycleOut_Velocity);
        if (request instanceof Diff_PositionDutyCycle_Velocity reqDiff_PositionDutyCycle_Velocity)
            return setControl(reqDiff_PositionDutyCycle_Velocity);
        if (request instanceof Diff_VelocityDutyCycle_Velocity reqDiff_VelocityDutyCycle_Velocity)
            return setControl(reqDiff_VelocityDutyCycle_Velocity);
        if (request instanceof Diff_MotionMagicDutyCycle_Velocity reqDiff_MotionMagicDutyCycle_Velocity)
            return setControl(reqDiff_MotionMagicDutyCycle_Velocity);
        if (request instanceof Diff_MotionMagicExpoDutyCycle_Velocity reqDiff_MotionMagicExpoDutyCycle_Velocity)
            return setControl(reqDiff_MotionMagicExpoDutyCycle_Velocity);
        if (request instanceof Diff_MotionMagicVelocityDutyCycle_Velocity reqDiff_MotionMagicVelocityDutyCycle_Velocity)
            return setControl(reqDiff_MotionMagicVelocityDutyCycle_Velocity);
        if (request instanceof Diff_DutyCycleOut_Open reqDiff_DutyCycleOut_Open)
            return setControl(reqDiff_DutyCycleOut_Open);
        if (request instanceof Diff_PositionDutyCycle_Open reqDiff_PositionDutyCycle_Open)
            return setControl(reqDiff_PositionDutyCycle_Open);
        if (request instanceof Diff_VelocityDutyCycle_Open reqDiff_VelocityDutyCycle_Open)
            return setControl(reqDiff_VelocityDutyCycle_Open);
        if (request instanceof Diff_MotionMagicDutyCycle_Open reqDiff_MotionMagicDutyCycle_Open)
            return setControl(reqDiff_MotionMagicDutyCycle_Open);
        if (request instanceof Diff_MotionMagicExpoDutyCycle_Open reqDiff_MotionMagicExpoDutyCycle_Open)
            return setControl(reqDiff_MotionMagicExpoDutyCycle_Open);
        if (request instanceof Diff_MotionMagicVelocityDutyCycle_Open reqDiff_MotionMagicVelocityDutyCycle_Open)
            return setControl(reqDiff_MotionMagicVelocityDutyCycle_Open);
        if (request instanceof Diff_VoltageOut_Position reqDiff_VoltageOut_Position)
            return setControl(reqDiff_VoltageOut_Position);
        if (request instanceof Diff_PositionVoltage_Position reqDiff_PositionVoltage_Position)
            return setControl(reqDiff_PositionVoltage_Position);
        if (request instanceof Diff_VelocityVoltage_Position reqDiff_VelocityVoltage_Position)
            return setControl(reqDiff_VelocityVoltage_Position);
        if (request instanceof Diff_MotionMagicVoltage_Position reqDiff_MotionMagicVoltage_Position)
            return setControl(reqDiff_MotionMagicVoltage_Position);
        if (request instanceof Diff_MotionMagicExpoVoltage_Position reqDiff_MotionMagicExpoVoltage_Position)
            return setControl(reqDiff_MotionMagicExpoVoltage_Position);
        if (request instanceof Diff_MotionMagicVelocityVoltage_Position reqDiff_MotionMagicVelocityVoltage_Position)
            return setControl(reqDiff_MotionMagicVelocityVoltage_Position);
        if (request instanceof Diff_VoltageOut_Velocity reqDiff_VoltageOut_Velocity)
            return setControl(reqDiff_VoltageOut_Velocity);
        if (request instanceof Diff_PositionVoltage_Velocity reqDiff_PositionVoltage_Velocity)
            return setControl(reqDiff_PositionVoltage_Velocity);
        if (request instanceof Diff_VelocityVoltage_Velocity reqDiff_VelocityVoltage_Velocity)
            return setControl(reqDiff_VelocityVoltage_Velocity);
        if (request instanceof Diff_MotionMagicVoltage_Velocity reqDiff_MotionMagicVoltage_Velocity)
            return setControl(reqDiff_MotionMagicVoltage_Velocity);
        if (request instanceof Diff_MotionMagicExpoVoltage_Velocity reqDiff_MotionMagicExpoVoltage_Velocity)
            return setControl(reqDiff_MotionMagicExpoVoltage_Velocity);
        if (request instanceof Diff_MotionMagicVelocityVoltage_Velocity reqDiff_MotionMagicVelocityVoltage_Velocity)
            return setControl(reqDiff_MotionMagicVelocityVoltage_Velocity);
        if (request instanceof Diff_VoltageOut_Open reqDiff_VoltageOut_Open)
            return setControl(reqDiff_VoltageOut_Open);
        if (request instanceof Diff_PositionVoltage_Open reqDiff_PositionVoltage_Open)
            return setControl(reqDiff_PositionVoltage_Open);
        if (request instanceof Diff_VelocityVoltage_Open reqDiff_VelocityVoltage_Open)
            return setControl(reqDiff_VelocityVoltage_Open);
        if (request instanceof Diff_MotionMagicVoltage_Open reqDiff_MotionMagicVoltage_Open)
            return setControl(reqDiff_MotionMagicVoltage_Open);
        if (request instanceof Diff_MotionMagicExpoVoltage_Open reqDiff_MotionMagicExpoVoltage_Open)
            return setControl(reqDiff_MotionMagicExpoVoltage_Open);
        if (request instanceof Diff_MotionMagicVelocityVoltage_Open reqDiff_MotionMagicVelocityVoltage_Open)
            return setControl(reqDiff_MotionMagicVelocityVoltage_Open);
        if (request instanceof Diff_TorqueCurrentFOC_Position reqDiff_TorqueCurrentFOC_Position)
            return setControl(reqDiff_TorqueCurrentFOC_Position);
        if (request instanceof Diff_PositionTorqueCurrentFOC_Position reqDiff_PositionTorqueCurrentFOC_Position)
            return setControl(reqDiff_PositionTorqueCurrentFOC_Position);
        if (request instanceof Diff_VelocityTorqueCurrentFOC_Position reqDiff_VelocityTorqueCurrentFOC_Position)
            return setControl(reqDiff_VelocityTorqueCurrentFOC_Position);
        if (request instanceof Diff_MotionMagicTorqueCurrentFOC_Position reqDiff_MotionMagicTorqueCurrentFOC_Position)
            return setControl(reqDiff_MotionMagicTorqueCurrentFOC_Position);
        if (request instanceof Diff_MotionMagicExpoTorqueCurrentFOC_Position reqDiff_MotionMagicExpoTorqueCurrentFOC_Position)
            return setControl(reqDiff_MotionMagicExpoTorqueCurrentFOC_Position);
        if (request instanceof Diff_MotionMagicVelocityTorqueCurrentFOC_Position reqDiff_MotionMagicVelocityTorqueCurrentFOC_Position)
            return setControl(reqDiff_MotionMagicVelocityTorqueCurrentFOC_Position);
        if (request instanceof Diff_TorqueCurrentFOC_Velocity reqDiff_TorqueCurrentFOC_Velocity)
            return setControl(reqDiff_TorqueCurrentFOC_Velocity);
        if (request instanceof Diff_PositionTorqueCurrentFOC_Velocity reqDiff_PositionTorqueCurrentFOC_Velocity)
            return setControl(reqDiff_PositionTorqueCurrentFOC_Velocity);
        if (request instanceof Diff_VelocityTorqueCurrentFOC_Velocity reqDiff_VelocityTorqueCurrentFOC_Velocity)
            return setControl(reqDiff_VelocityTorqueCurrentFOC_Velocity);
        if (request instanceof Diff_MotionMagicTorqueCurrentFOC_Velocity reqDiff_MotionMagicTorqueCurrentFOC_Velocity)
            return setControl(reqDiff_MotionMagicTorqueCurrentFOC_Velocity);
        if (request instanceof Diff_MotionMagicExpoTorqueCurrentFOC_Velocity reqDiff_MotionMagicExpoTorqueCurrentFOC_Velocity)
            return setControl(reqDiff_MotionMagicExpoTorqueCurrentFOC_Velocity);
        if (request instanceof Diff_MotionMagicVelocityTorqueCurrentFOC_Velocity reqDiff_MotionMagicVelocityTorqueCurrentFOC_Velocity)
            return setControl(reqDiff_MotionMagicVelocityTorqueCurrentFOC_Velocity);
        if (request instanceof Diff_TorqueCurrentFOC_Open reqDiff_TorqueCurrentFOC_Open)
            return setControl(reqDiff_TorqueCurrentFOC_Open);
        if (request instanceof Diff_PositionTorqueCurrentFOC_Open reqDiff_PositionTorqueCurrentFOC_Open)
            return setControl(reqDiff_PositionTorqueCurrentFOC_Open);
        if (request instanceof Diff_VelocityTorqueCurrentFOC_Open reqDiff_VelocityTorqueCurrentFOC_Open)
            return setControl(reqDiff_VelocityTorqueCurrentFOC_Open);
        if (request instanceof Diff_MotionMagicTorqueCurrentFOC_Open reqDiff_MotionMagicTorqueCurrentFOC_Open)
            return setControl(reqDiff_MotionMagicTorqueCurrentFOC_Open);
        if (request instanceof Diff_MotionMagicExpoTorqueCurrentFOC_Open reqDiff_MotionMagicExpoTorqueCurrentFOC_Open)
            return setControl(reqDiff_MotionMagicExpoTorqueCurrentFOC_Open);
        if (request instanceof Diff_MotionMagicVelocityTorqueCurrentFOC_Open reqDiff_MotionMagicVelocityTorqueCurrentFOC_Open)
            return setControl(reqDiff_MotionMagicVelocityTorqueCurrentFOC_Open);
        return StatusCode.NotSupported;
    }

    
    /**
     * Sets the mechanism position of the device in mechanism rotations.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @param newValue Value to set to. Units are in rotations.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode setPosition(double newValue) {
        return setPosition(newValue, 0.100);
    }
    /**
     * Sets the mechanism position of the device in mechanism rotations.
     * 
     * @param newValue Value to set to. Units are in rotations.
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode setPosition(double newValue, double timeoutSeconds) {
        return getConfigurator().setPosition(newValue, timeoutSeconds);
    }
    
    /**
     * Sets the mechanism position of the device in mechanism rotations.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @param newValue Value to set to. Units are in rotations.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode setPosition(Angle newValue) {
        return setPosition(newValue.in(Rotations));
    }
    /**
     * Sets the mechanism position of the device in mechanism rotations.
     * 
     * @param newValue Value to set to. Units are in rotations.
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode setPosition(Angle newValue, double timeoutSeconds) {
        return setPosition(newValue.in(Rotations), timeoutSeconds);
    }
    
    /**
     * Clear the sticky faults in the device.
     * <p>
     * This typically has no impact on the device functionality.  Instead,
     * it just clears telemetry faults that are accessible via API and
     * Tuner Self-Test.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFaults() {
        return clearStickyFaults(0.100);
    }
    /**
     * Clear the sticky faults in the device.
     * <p>
     * This typically has no impact on the device functionality.  Instead,
     * it just clears telemetry faults that are accessible via API and
     * Tuner Self-Test.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFaults(double timeoutSeconds) {
        return getConfigurator().clearStickyFaults(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Hardware fault occurred
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_Hardware() {
        return clearStickyFault_Hardware(0.100);
    }
    /**
     * Clear sticky fault: Hardware fault occurred
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_Hardware(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_Hardware(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Processor temperature exceeded limit
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ProcTemp() {
        return clearStickyFault_ProcTemp(0.100);
    }
    /**
     * Clear sticky fault: Processor temperature exceeded limit
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ProcTemp(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_ProcTemp(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Device temperature exceeded limit
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_DeviceTemp() {
        return clearStickyFault_DeviceTemp(0.100);
    }
    /**
     * Clear sticky fault: Device temperature exceeded limit
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_DeviceTemp(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_DeviceTemp(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Device supply voltage dropped to near brownout
     * levels
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_Undervoltage() {
        return clearStickyFault_Undervoltage(0.100);
    }
    /**
     * Clear sticky fault: Device supply voltage dropped to near brownout
     * levels
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_Undervoltage(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_Undervoltage(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Device boot while detecting the enable signal
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_BootDuringEnable() {
        return clearStickyFault_BootDuringEnable(0.100);
    }
    /**
     * Clear sticky fault: Device boot while detecting the enable signal
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_BootDuringEnable(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_BootDuringEnable(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: An unlicensed feature is in use, device may not
     * behave as expected.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_UnlicensedFeatureInUse() {
        return clearStickyFault_UnlicensedFeatureInUse(0.100);
    }
    /**
     * Clear sticky fault: An unlicensed feature is in use, device may not
     * behave as expected.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_UnlicensedFeatureInUse(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_UnlicensedFeatureInUse(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Bridge was disabled most likely due to supply
     * voltage dropping too low.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_BridgeBrownout() {
        return clearStickyFault_BridgeBrownout(0.100);
    }
    /**
     * Clear sticky fault: Bridge was disabled most likely due to supply
     * voltage dropping too low.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_BridgeBrownout(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_BridgeBrownout(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: The remote sensor has reset.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_RemoteSensorReset() {
        return clearStickyFault_RemoteSensorReset(0.100);
    }
    /**
     * Clear sticky fault: The remote sensor has reset.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_RemoteSensorReset(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_RemoteSensorReset(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: The remote Talon used for differential control
     * is not present on CAN Bus.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_MissingDifferentialFX() {
        return clearStickyFault_MissingDifferentialFX(0.100);
    }
    /**
     * Clear sticky fault: The remote Talon used for differential control
     * is not present on CAN Bus.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_MissingDifferentialFX(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_MissingDifferentialFX(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: The remote sensor position has overflowed.
     * Because of the nature of remote sensors, it is possible for the
     * remote sensor position to overflow beyond what is supported by the
     * status signal frame. However, this is rare and cannot occur over
     * the course of an FRC match under normal use.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_RemoteSensorPosOverflow() {
        return clearStickyFault_RemoteSensorPosOverflow(0.100);
    }
    /**
     * Clear sticky fault: The remote sensor position has overflowed.
     * Because of the nature of remote sensors, it is possible for the
     * remote sensor position to overflow beyond what is supported by the
     * status signal frame. However, this is rare and cannot occur over
     * the course of an FRC match under normal use.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_RemoteSensorPosOverflow(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_RemoteSensorPosOverflow(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Supply Voltage has exceeded the maximum voltage
     * rating of device.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_OverSupplyV() {
        return clearStickyFault_OverSupplyV(0.100);
    }
    /**
     * Clear sticky fault: Supply Voltage has exceeded the maximum voltage
     * rating of device.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_OverSupplyV(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_OverSupplyV(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Supply Voltage is unstable.  Ensure you are
     * using a battery and current limited power supply.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_UnstableSupplyV() {
        return clearStickyFault_UnstableSupplyV(0.100);
    }
    /**
     * Clear sticky fault: Supply Voltage is unstable.  Ensure you are
     * using a battery and current limited power supply.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_UnstableSupplyV(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_UnstableSupplyV(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Reverse limit switch has been asserted.  Output
     * is set to neutral.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ReverseHardLimit() {
        return clearStickyFault_ReverseHardLimit(0.100);
    }
    /**
     * Clear sticky fault: Reverse limit switch has been asserted.  Output
     * is set to neutral.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ReverseHardLimit(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_ReverseHardLimit(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Forward limit switch has been asserted.  Output
     * is set to neutral.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ForwardHardLimit() {
        return clearStickyFault_ForwardHardLimit(0.100);
    }
    /**
     * Clear sticky fault: Forward limit switch has been asserted.  Output
     * is set to neutral.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ForwardHardLimit(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_ForwardHardLimit(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Reverse soft limit has been asserted.  Output
     * is set to neutral.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ReverseSoftLimit() {
        return clearStickyFault_ReverseSoftLimit(0.100);
    }
    /**
     * Clear sticky fault: Reverse soft limit has been asserted.  Output
     * is set to neutral.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ReverseSoftLimit(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_ReverseSoftLimit(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Forward soft limit has been asserted.  Output
     * is set to neutral.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ForwardSoftLimit() {
        return clearStickyFault_ForwardSoftLimit(0.100);
    }
    /**
     * Clear sticky fault: Forward soft limit has been asserted.  Output
     * is set to neutral.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_ForwardSoftLimit(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_ForwardSoftLimit(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: The remote soft limit device is not present on
     * CAN Bus.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_MissingSoftLimitRemote() {
        return clearStickyFault_MissingSoftLimitRemote(0.100);
    }
    /**
     * Clear sticky fault: The remote soft limit device is not present on
     * CAN Bus.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_MissingSoftLimitRemote(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_MissingSoftLimitRemote(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: The remote limit switch device is not present
     * on CAN Bus.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_MissingHardLimitRemote() {
        return clearStickyFault_MissingHardLimitRemote(0.100);
    }
    /**
     * Clear sticky fault: The remote limit switch device is not present
     * on CAN Bus.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_MissingHardLimitRemote(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_MissingHardLimitRemote(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: The remote sensor's data is no longer trusted.
     * This can happen if the remote sensor disappears from the CAN bus or
     * if the remote sensor indicates its data is no longer valid, such as
     * when a CANcoder's magnet strength falls into the "red" range.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_RemoteSensorDataInvalid() {
        return clearStickyFault_RemoteSensorDataInvalid(0.100);
    }
    /**
     * Clear sticky fault: The remote sensor's data is no longer trusted.
     * This can happen if the remote sensor disappears from the CAN bus or
     * if the remote sensor indicates its data is no longer valid, such as
     * when a CANcoder's magnet strength falls into the "red" range.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_RemoteSensorDataInvalid(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_RemoteSensorDataInvalid(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: The remote sensor used for fusion has fallen
     * out of sync to the local sensor. A re-synchronization has occurred,
     * which may cause a discontinuity. This typically happens if there is
     * significant slop in the mechanism, or if the RotorToSensorRatio
     * configuration parameter is incorrect.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_FusedSensorOutOfSync() {
        return clearStickyFault_FusedSensorOutOfSync(0.100);
    }
    /**
     * Clear sticky fault: The remote sensor used for fusion has fallen
     * out of sync to the local sensor. A re-synchronization has occurred,
     * which may cause a discontinuity. This typically happens if there is
     * significant slop in the mechanism, or if the RotorToSensorRatio
     * configuration parameter is incorrect.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_FusedSensorOutOfSync(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_FusedSensorOutOfSync(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Stator current limit occured.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_StatorCurrLimit() {
        return clearStickyFault_StatorCurrLimit(0.100);
    }
    /**
     * Clear sticky fault: Stator current limit occured.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_StatorCurrLimit(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_StatorCurrLimit(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Supply current limit occured.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_SupplyCurrLimit() {
        return clearStickyFault_SupplyCurrLimit(0.100);
    }
    /**
     * Clear sticky fault: Supply current limit occured.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_SupplyCurrLimit(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_SupplyCurrLimit(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Using Fused CANcoder feature while unlicensed.
     * Device has fallen back to remote CANcoder.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_UsingFusedCANcoderWhileUnlicensed() {
        return clearStickyFault_UsingFusedCANcoderWhileUnlicensed(0.100);
    }
    /**
     * Clear sticky fault: Using Fused CANcoder feature while unlicensed.
     * Device has fallen back to remote CANcoder.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_UsingFusedCANcoderWhileUnlicensed(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_UsingFusedCANcoderWhileUnlicensed(timeoutSeconds);
    }
    
    /**
     * Clear sticky fault: Static brake was momentarily disabled due to
     * excessive braking current while disabled.
     * <p>
     * This will wait up to 0.100 seconds (100ms) by default.
     * 
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_StaticBrakeDisabled() {
        return clearStickyFault_StaticBrakeDisabled(0.100);
    }
    /**
     * Clear sticky fault: Static brake was momentarily disabled due to
     * excessive braking current while disabled.
     * 
     * @param timeoutSeconds Maximum time to wait up to in seconds.
     * @return StatusCode of the set command
     */
    @Override
    public final StatusCode clearStickyFault_StaticBrakeDisabled(double timeoutSeconds) {
        return getConfigurator().clearStickyFault_StaticBrakeDisabled(timeoutSeconds);
    }
}

