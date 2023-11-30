// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TITANIUM RAMS 5959, RAMPAGE
// by: Sebastián León

// cosas comentareadas:
// "OR" de los cíclicos con operador
// control de la garra con operador

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;


//  ================= 
//       Red CAN
//  === ============= 
//   0 · PDP
//   1 · PCM
//   2 · rearRight
//   3 · rearLeft
//   4 · frontRight
//   5 · frontLeft
//   6 · Extensor
//   7 · Garra
//   8 · Elevador
//  === =============


public class Robot extends TimedRobot {

  private final Timer cronos = new Timer();

  // LEDS

  private static boolean ledsCOMP = true;

  private static AddressableLED INFO_led;
  private static AddressableLEDBuffer INFO_buffer;
  private static int INFO_rfph; // Store what the last hue of the first pixel is

  private static AddressableLED BODY_led;
  private static AddressableLEDBuffer BODY_buffer;
  private static int BODY_rfph;

  private AHRS navx;
  private MecanumDrive m_robotDrive;
  private double TeleopPotencia;
  private PS4Controller control;
  private GenericHID operador;
  private double autoChoose;

  // variables tiempo auto
  private double AutoPausa;
  private double AutoTiempo_A, AutoTiempo_B, AutoTiempo_C, AutoTiempo_D, AutoTiempo_E, AutoTiempo_F, AutoTiempo_G;
  private boolean autoDone = false;

  // MOS PID stuff
  private PIDController turnController;
  private double Despl_X, Despl_Y, Rotation_Z;

  // controladores de motores
  public VictorSP VicPWM_0, VicPWM_1, VicPWM_2, VicPWM_3;
  public static WPI_VictorSPX frontLeft, rearLeft, frontRight, rearRight;

  // limelight datos
  NetworkTable datosLimelight = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = datosLimelight.getEntry("tx");
  NetworkTableEntry ty = datosLimelight.getEntry("ty");
  NetworkTableEntry ta = datosLimelight.getEntry("ta");
  NetworkTableEntry tv = datosLimelight.getEntry("tv");

  NetworkTableEntry LL_leds = datosLimelight.getEntry("ledMode");
  NetworkTableEntry LL_pipe = datosLimelight.getEntry("pipeline");

  private double x_Limelight, y_Limelight, a_limelight;
  private double t_limelight;

  // neos
  private CANSparkMax extensorNeo, garraCIM, elevadorNeo;
  private RelativeEncoder encoderExtnsr, encoderElevador;
  private double posExtensor, posElevador;
  private double outputExtensor, outputGarra, outputElevador;
  private double absClampExtnsr, absClampElevador;
  private double velExtensor, velElevador;
  private double relacionExtensor = 25;
  private double relacionGarra = 100;
  private double relacionElevador = 100;
  private int setSwitchExtensor = 0;
  private int setSwitchElevador = 0;
  private boolean extensorPID_switch, elevadorPID_switch;

  //PID de los Neo
  private PIDController extensorPID, elevadorPID;

  // intake
  private MotorControllerGroup intakeGroup;
  private double potenciaIntake;

  // pneumática
  private final Compressor comp = new Compressor(1, PneumaticsModuleType.CTREPCM);
  private final boolean pressureSwitch = comp.getPressureSwitchValue();
  private final DoubleSolenoid dSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 1, 2);

  @Override
  public void robotInit() {

    // objetos de los victors SPX
    frontLeft = new WPI_VictorSPX(5);
    rearLeft = new WPI_VictorSPX(3);
    frontRight = new WPI_VictorSPX(4);
    rearRight = new WPI_VictorSPX(2);

    // invertir los necesarios
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    // objeto de la mecanum
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    // objetos de los victor SP (pwm)
    VicPWM_0 = new VictorSP(0);
    VicPWM_1 = new VictorSP(1);
    VicPWM_2 = new VictorSP(2);
    VicPWM_3 = new VictorSP(3);

    // PID MOS ángulo
    turnController = new PIDController(0.01, 0, 0);
    turnController.enableContinuousInput(-180, 180);

    // PID neos
    extensorPID = new PIDController(0.015, 0, 0);
    elevadorPID = new PIDController(0.015, 0, 0);

    // neos
    extensorNeo = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    extensorNeo.setInverted(true);
    encoderExtnsr = extensorNeo.getEncoder();
    encoderExtnsr.setPosition(0);

    garraCIM = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushed); //este es un cim
    garraCIM.setInverted(true);

    elevadorNeo = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevadorNeo.setInverted(true);
    encoderElevador = elevadorNeo.getEncoder();
    encoderElevador.setPosition(0);

    // desactivar el brake
    chasisBrake(false);

    // apagar los leds de la LimeLight
    LL_leds.setNumber(1);

    // objetos...
    navx = new AHRS(SPI.Port.kMXP);
    control = new PS4Controller(0);
    operador = new GenericHID(1);

    // intake
    intakeGroup = new MotorControllerGroup(VicPWM_0, VicPWM_1);
    VicPWM_0.setInverted(true);
    VicPWM_1.setInverted(false);

    // neumática
    comp.isEnabled();
    dSolenoid.set(Value.kForward);

    // -------------SMARTDASHBOARD-------------
    CameraServer.startAutomaticCapture(); //mandar video de webcam

    SmartDashboard.putNumber("Potencia", 0.90);
    SmartDashboard.putBoolean("Field Oriented Drive", true);
    SmartDashboard.putNumber("Autonomus", 0);

    // entradas tiempo de autónomos
    SmartDashboard.putNumber("Pausa Auto", 0);
    SmartDashboard.putNumber("Tiempo A", 2);
    SmartDashboard.putNumber("Tiempo B", 1);
    SmartDashboard.putNumber("Tiempo C", 3);
    SmartDashboard.putNumber("Tiempo D", 1.5);
    SmartDashboard.putNumber("Tiempo E", 1);
    SmartDashboard.putNumber("Tiempo F", 2);
    SmartDashboard.putNumber("Tiempo G", 5);
 

    // PID del ángulo
    SmartDashboard.putNumber("P", 0.01);
    SmartDashboard.putNumber("I", 0.0);
    SmartDashboard.putNumber("D", 0.0);

    // Velocidad del Intake
    SmartDashboard.putNumber("Intake IN", 0.5);
    SmartDashboard.putNumber("Intake OUT", 0.7);
    SmartDashboard.putNumber("Garra Subir", 0.5);
    SmartDashboard.putNumber("Garra Bajar", 0.5);

    // -Neos-------------------------------------------------------

    // Extensor 25:1
    SmartDashboard.putNumber("Relacion Extnsr", relacionExtensor);
    SmartDashboard.putNumber("Vueltas Extnsr", 1);
    SmartDashboard.putNumber("Clamp Extnsr", 0.50);
    SmartDashboard.putNumber("Position Extnsr", 0);
    SmartDashboard.putNumber("Vel. Extnsr", 0);
    SmartDashboard.putBoolean("Extensor Reset", false);
    SmartDashboard.putNumber("Pot. Manual Extensor", 0.2);
    
    // Garra 100:1
    SmartDashboard.putNumber("Relacion Garra", relacionGarra);
    SmartDashboard.putNumber("Garra Subir", 0.5);
    SmartDashboard.putNumber("Garra Bajar", 0.5);

    // Elevador 100:1
    SmartDashboard.putNumber("Relacion Elevador", relacionElevador);
    SmartDashboard.putNumber("Vueltas Elevador", 1);
    SmartDashboard.putNumber("Clamp Elevador", 0.85);
    SmartDashboard.putNumber("Position Elevador", 0);
    SmartDashboard.putNumber("Vel. Elevador", 0);
    SmartDashboard.putBoolean("Elevador Reset", false);
    SmartDashboard.putNumber("Pot. Manual Elevador", 0.2);

    // LEDS
    if (ledsCOMP){
      INFO_led = new AddressableLED(9);
      INFO_buffer = new AddressableLEDBuffer(7);
      INFO_led.setLength(INFO_buffer.getLength());
      INFO_led.setData(INFO_buffer);
      INFO_led.start();
    }
    else {
      BODY_led = new AddressableLED(8);
      BODY_buffer = new AddressableLEDBuffer(300);
      BODY_led.setLength(BODY_buffer.getLength());
      BODY_led.setData(BODY_buffer);
      BODY_led.start();
    }
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Voltaje", RobotController.getBatteryVoltage());

    if (!ledsCOMP){
      // arcoíris BODY
      for (var i = 0; i < BODY_buffer.getLength(); i++) {
        final var hue = (BODY_rfph + (i * 180 / BODY_buffer.getLength())) % 180;
        BODY_buffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      BODY_rfph += 4;
      // Check bounds
      BODY_rfph %= 180;
    
      BODY_led.setData(BODY_buffer);
    }
  }

  @Override
  public void autonomousInit() {

    //pasamos el switch de confirmación de que ya se hizo la sequencia autónoma
    autoDone = true;

    // neos
    extensorNeo.setIdleMode(IdleMode.kBrake);
    encoderExtnsr.setPosition(0);

    garraCIM.setIdleMode(IdleMode.kBrake);

    elevadorNeo.setIdleMode(IdleMode.kBrake);
    encoderElevador.setPosition(0);

    // encendemos el frenado de las llantas
    chasisBrake(true);

    // cerrar garra
    dSolenoid.set(Value.kForward);

    // asignando valores de tiempo a las variables
    autoChoose = SmartDashboard.getNumber("Autonomus", 0);

    //poner tiempos
    AutoPausa = SmartDashboard.getNumber("Pausa Auto", 0);

    AutoTiempo_A = SmartDashboard.getNumber("Tiempo A", 2);    //Bajar Garra  
    AutoTiempo_B = SmartDashboard.getNumber("Tiempo B", 1);    //Abrir Garra
    AutoTiempo_C = SmartDashboard.getNumber("Tiempo C", 3);    //Mover Atras
    AutoTiempo_D = SmartDashboard.getNumber("Tiempo D", 1.5);  //Mover izq/der
    AutoTiempo_E = SmartDashboard.getNumber("Tiempo E", 3.5);    //Mover elevador/extensor
    AutoTiempo_F = SmartDashboard.getNumber("Tiempo F", 2);    //Mover izq/der ligeramente
    AutoTiempo_G = SmartDashboard.getNumber("Tiempo G", 1.5);    //Subir garra
    // tiempo total = 14.5 segundos

    cronos.reset();
    cronos.start();
  }

  @Override
  public void autonomousPeriodic() {

    // NEOS: sacar datos del dashboard
    absClampExtnsr = SmartDashboard.getNumber("Clamp Extnsr", 0);
    absClampElevador = SmartDashboard.getNumber("Clamp Elevador", 0);

    // NEOS: sacar datos de encoders
    posExtensor = encoderExtnsr.getPosition();
    posElevador = encoderElevador.getPosition();

    // #1 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Dejar elemento y salir hacia atras
    if ((autoChoose == 1) || (autoChoose == 3)){
      if (cronos.get() < AutoPausa){ //delay para empezar
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0; // ceros
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      if (cronos.get() < AutoPausa + AutoTiempo_A) { //bajar garra  
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        potenciaIntake = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoPausa + AutoTiempo_A + AutoTiempo_B) { //abrir garra  
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = -0.5; //disparar cubo
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      }
      else if (cronos.get() < AutoPausa + AutoTiempo_A + AutoTiempo_B + AutoTiempo_C) { //salir atrás  
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0;
        Despl_Y = -0.5; //atrás
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0; // ceros
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
    } //Fin de autonomo 1

    // #1.2 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (autoChoose == 1.2){
      if (cronos.get() < AutoPausa){ //delay para empezar
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0; // ceros
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      if (cronos.get() < AutoPausa + AutoTiempo_E) { // subir elevador
        outputElevador = MathUtil.clamp(elevadorPID.calculate(posElevador, 290), -absClampElevador, absClampElevador);
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoPausa + AutoTiempo_E + AutoTiempo_A) { // bajar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; //bajarla
        potenciaIntake = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoPausa + AutoTiempo_E + AutoTiempo_A + AutoTiempo_B) { // abrir garra y expulsar
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = -0.7; // expulsar
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      }
      else if (cronos.get() < AutoPausa + AutoTiempo_E + AutoTiempo_A + AutoTiempo_B + AutoTiempo_G) { // subir y cerrar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0.5; // levantarla
        potenciaIntake = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kForward); // cerrar
      }
      else if (cronos.get() < AutoPausa + AutoTiempo_E + AutoTiempo_A + AutoTiempo_B + AutoTiempo_G + AutoTiempo_F) { // moverse de lado
        outputElevador = MathUtil.clamp(elevadorPID.calculate(posElevador, 0), -absClampElevador, absClampElevador);
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0;
        Despl_Y = 0;
        Despl_X = -0.7; // moverse izquierda
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoPausa + AutoTiempo_E + AutoTiempo_A + AutoTiempo_B + AutoTiempo_G + AutoTiempo_F + AutoTiempo_C) { // salir de comunidad
        outputElevador = MathUtil.clamp(elevadorPID.calculate(posElevador, 0), -absClampElevador, absClampElevador);
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0;
        Despl_Y = -0.5; // reversa
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        potenciaIntake = 0; // ceros
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
    }

    // #2 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (autoChoose == 2){
      if (cronos.get() < AutoTiempo_A) { //bajar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B) { // abrir garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_G) { // para motor
                outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0; //izquierda
        Rotation_Z = 0;        
      } 
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_G + AutoTiempo_D) { // moverse izquierda (Field Relative)
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = -0.7; //izquierda
        Rotation_Z = 0;        
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_G + AutoTiempo_D + AutoTiempo_C) {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = -0.5; // reversa
        Despl_X = 0;
        Rotation_Z = 0;    
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0; 
        Despl_Y = 0;     //ceros
        Despl_X = 0;
        Rotation_Z = 0;
      }
    }

    // #3 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (autoChoose == 3){
      if (cronos.get() < AutoTiempo_A) { //bajar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B) { // abrir garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      } 
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_G) { // para motor
        outputElevador = 0;
outputExtensor = 0;
outputGarra = 0;
Despl_Y = 0;
Despl_X = 0; //izquierda
Rotation_Z = 0;        
}
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_G + AutoTiempo_D) { // moverse derecha (Field Relative)
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0.7; // derecha
        Rotation_Z = 0;        
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_G + AutoTiempo_D + AutoTiempo_C) {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = -0.5; // reversa
        Despl_X = 0;
        Rotation_Z = 0;    
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0; 
        Despl_Y = 0;     //ceros
        Despl_X = 0;
        Rotation_Z = 0;
      }
    }

    // #4 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (autoChoose == 4){
      if (cronos.get() < AutoTiempo_A) { //bajar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B) { //abrir garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_C) { //salir atrás
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = -0.5; //atrás
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_C + AutoTiempo_D) { //moverse derecha, field relative
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0.7; // derecha
        Rotation_Z = 0;   
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_C + AutoTiempo_D + AutoTiempo_E) { // dock charging station
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0.5; //adelante
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0; 
        Despl_Y = 0;     //ceros
        Despl_X = 0;
        Rotation_Z = 0;
      }
    }

    // #5 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (autoChoose == 5){
      if (cronos.get() < AutoTiempo_A) { //bajar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B) { //abrir garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_C) { //salir atrás
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = -0.5; //atrás
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_C + AutoTiempo_D) { //moverse izquierda, field relative
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = -0.7; // izquierda
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_C + AutoTiempo_D + AutoTiempo_E) {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0.5; //adelante
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0; 
        Despl_Y = 0;     //ceros
        Despl_X = 0;
        Rotation_Z = 0;
      }
    }

    // #6 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (autoChoose == 6){
      if (cronos.get() < AutoTiempo_A) { //bajar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B) { // abrir garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      } 
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D) { // moverse izquierda (Field Relative)
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = -0.7; //izquierda
        Rotation_Z = 0;        
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D + AutoTiempo_C) { // salir atrás
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = -0.5; // reversa
        Despl_X = 0;
        Rotation_Z = 0;    
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D + AutoTiempo_C + AutoTiempo_D) { //moverse derecha, field relative
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0.7; // derecha
        Rotation_Z = 0;   
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D + AutoTiempo_C + AutoTiempo_D + AutoTiempo_E) { // dock charging station
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0.5; //adelante
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0; 
        Despl_Y = 0;     //ceros
        Despl_X = 0;
        Rotation_Z = 0;
      }
    }

    // #7 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    else if (autoChoose == 7){
      if (cronos.get() < AutoTiempo_A) { //bajar garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B) { // abrir garra
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      } 
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D) { // moverse derecha (Field Relative)
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0.7; //derecha
        Rotation_Z = 0;        
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D + AutoTiempo_C) { // salir atrás
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = -0.5; // reversa
        Despl_X = 0;
        Rotation_Z = 0;    
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D + AutoTiempo_C + AutoTiempo_D) { //moverse izquierda, field relative
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = -0.7; // izquierda
        Rotation_Z = 0;   
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_D + AutoTiempo_C + AutoTiempo_D + AutoTiempo_E) { // dock charging station
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0.5; //adelante
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;   //ceros
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
    }

    // #8 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Dejar elemento y salir hacia atras
    else if (autoChoose == 8){
      if (cronos.get() < AutoTiempo_A) { //bajar garra  
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = -0.5; // in
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B) { //abrir garra  
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = 0;
        Despl_X = 0;
        Rotation_Z = 0;
        dSolenoid.set(Value.kReverse); // abrir
      }
      else if (cronos.get() < AutoTiempo_A + AutoTiempo_B + AutoTiempo_F) { //salir atrás  
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0;
        Despl_Y = -0.5; //atrás
        Despl_X = 0;
        Rotation_Z = 0;
      }
      else {
        outputElevador = 0;
        outputExtensor = 0;
        outputGarra = 0; 
        Despl_Y = 0;     //ceros
        Despl_X = 0;
        Rotation_Z = 0;
      }
    } //Fin de autonomo 8

    // mandar señal a motores
    elevadorNeo.set(outputElevador);
    extensorNeo.set(outputExtensor);
    garraCIM.set(outputGarra);
    intakeGroup.set(potenciaIntake);
    m_robotDrive.driveCartesian(Despl_Y, Despl_X, Rotation_Z);
  }

  @Override
  public void teleopInit() {

    // en caso de no haber pasado por periodo autónomo, mandamos todo a inicio
    if (!autoDone){
      // neos
      extensorNeo.setIdleMode(IdleMode.kBrake);
      setSwitchExtensor = 0;
      encoderExtnsr.setPosition(0);
      extensorPID_switch = true;

      garraCIM.setIdleMode(IdleMode.kBrake);

      elevadorNeo.setIdleMode(IdleMode.kBrake);
      setSwitchElevador = 0;
      encoderElevador.setPosition(0);
      elevadorPID_switch = true;

      // ponemos las llantas en Brake, no se moverán por inercia
      chasisBrake(true);
    }

    // ponemos leds según la alianza
    if (ledsCOMP){
      if (DriverStation.getAlliance() == Alliance.Blue){ // alianza azul
        for (var i = 0; i < INFO_buffer.getLength(); i++) { //leds info
          INFO_buffer.setRGB(i, 0, 0, 255);
        }
        INFO_led.setData(INFO_buffer);
  
      }
      else{ // alianza roja
        for (var i = 0; i < INFO_buffer.getLength(); i++) { //leds info
          INFO_buffer.setRGB(i, 255, 0, 0);
        }
        INFO_led.setData(INFO_buffer);
      }
    }

    //apagar leds LimeLight
    LL_leds.setNumber(1);

    cronos.reset();
    cronos.start();
  }

  @Override
  public void teleopPeriodic() {

    GyroData(); // ponemos datos de la navx en el smartdashboard  

    // Limelight: read values periodically
    x_Limelight = tx.getDouble(0.0);
    y_Limelight = ty.getDouble(0.0);
    a_limelight = ta.getDouble(0.0);
    t_limelight = tv.getDouble(0.0);

    // afinadas de PID MOS
    turnController.setP(SmartDashboard.getNumber("P", 0.0));
    turnController.setI(SmartDashboard.getNumber("I", 0.0));
    turnController.setD(SmartDashboard.getNumber("D", 0.0));

    // NEOS: sacar datos de encoders
    posExtensor = encoderExtnsr.getPosition();
    posElevador = encoderElevador.getPosition();

    velExtensor = encoderExtnsr.getVelocity();
    velElevador = encoderElevador.getVelocity();

    // NEOS: mandar datos al dashboard
    SmartDashboard.putNumber("Position Extnsr" , posExtensor);
    SmartDashboard.putNumber("Position Elevador" , posElevador);

    SmartDashboard.putNumber("Vel. Extnsr", velExtensor);
    SmartDashboard.putNumber("Vel. Elevador", velElevador);

    // NEOS: sacar datos del dashboard
    absClampExtnsr = SmartDashboard.getNumber("Clamp Extnsr", 0);
    absClampElevador = SmartDashboard.getNumber("Clamp Elevador", 0);

    // ------------------------------------MECANUM DRIVE------------------------------------
    TeleopPotencia = SmartDashboard.getNumber("Potencia", 0.7); // saca el valor de la potencia del dashboard

    Despl_X = control.getLeftX() * TeleopPotencia;
    Despl_Y = control.getLeftY() * TeleopPotencia;
    Rotation_Z = control.getRightX() * TeleopPotencia; // asignar el valor del joystick

    if (SmartDashboard.getBoolean("Field Oriented Drive", true)) {

      // pid de la rotación a un ángulo determinado, se activa si una cruzeta está presionada
      if (control.getL3Button()) {
        turnController.setSetpoint(0);
        // cálculo del PID, se hace override al input del control
        Rotation_Z = turnController.calculate(navx.getYaw());
      }
      else {
        turnController.reset(); // mientras no se usa, se limpia y resetea
      }

      m_robotDrive.driveCartesian(
        -Despl_Y,   // Y NOTA: ES MUY NECESARIO PONER ESTE EN NEGATIVO
        Despl_X,    // X
        Rotation_Z, // Z
        Rotation2d.fromDegrees(navx.getAngle()) ); // Gyro
    }

    else {
      m_robotDrive.driveCartesian(
        -Despl_Y,    // Y NOTA: ES MUY NECESARIO PONER ESTE EN NEGATIVO
        Despl_X,     // X
        Rotation_Z); // Z
    }

    ////////////////////////////////////////////////////////////////////////////////////////

    // ------------------------------------ NEOS ------------------------------------ //

    // Reseteamos TODOS los setpoints en 0
    if (control.getPSButtonPressed()){
      setSwitchExtensor = 0;
      setSwitchElevador = 0;

      extensorPID_switch = true;
      elevadorPID_switch = true;
    }

    // Resetear posiciones con botón del dashboard
    if (SmartDashboard.getBoolean("Extensor Reset", false)){
      extensorPID_switch = false;
      setSwitchExtensor = 0;
      encoderExtnsr.setPosition(0);
    }
    if (SmartDashboard.getBoolean("Elevador Reset", false)){
      elevadorPID_switch = false;
      setSwitchElevador = 0;
      encoderElevador.setPosition(0);
    }

    // --ELEVADOR-- //

    // poner setpoint, cíclico
    if (control.getTriangleButtonPressed() /*|| operador.getRawButtonPressed(4) */){
      elevadorPID_switch = true;
      if (setSwitchElevador == 0){
        setSwitchElevador = 290; //posición media
      }
      else if(setSwitchElevador == 290){
        setSwitchElevador = 460; //posición top
      }
      else if (setSwitchElevador == 460){
        setSwitchElevador = 0;
      }
    }

    // calcular output en base al setpoint
    if (elevadorPID_switch){
      outputElevador = MathUtil.clamp(elevadorPID.calculate(posElevador, setSwitchElevador), -absClampElevador, absClampElevador);
    }

    //control manual del elevador; limite touch
    if (control.getTouchpad()){
      elevadorPID_switch = false;
      if (control.getPOV() == 0){
        outputElevador = SmartDashboard.getNumber("Pot. Manual Elevador", 0);
      }
      else if (control.getPOV() == 180){
        outputElevador = -SmartDashboard.getNumber("Pot. Manual Elevador", 0);
      }
      else{
        outputElevador = 0;
      }
    }
    else if (control.getPOV() == 0){
      elevadorPID_switch = false;
      outputElevador = SmartDashboard.getNumber("Pot. Manual Elevador", 0);
      if (posElevador > 460){
        outputElevador = MathUtil.clamp(outputElevador, -1, 0);
      }
    }
    else if (control.getPOV() == 180){
      elevadorPID_switch = false;
      outputElevador = -SmartDashboard.getNumber("Pot. Manual Elevador", 0);
      if (posElevador < -2){
        outputElevador = MathUtil.clamp(outputElevador, 0, 1);
      }
    }
    else if ((control.getPOV() != 0 && control.getPOV() != 180) && !elevadorPID_switch){
      outputElevador = 0;
    }

    // mandar output después de las condiciones
    elevadorNeo.set(outputElevador);

    // --EXTENSOR-- //

    // poner setpoint, cíclico
    if (control.getCircleButtonPressed() /*|| operador.getRawButtonPressed(2)*/ ){
      extensorPID_switch = true;
      if (setSwitchExtensor == 0){
        setSwitchExtensor = 50; //posición media
      }
      else if(setSwitchExtensor == 50){
        setSwitchExtensor = 0; //posición top
      }
    }

    // calcular output en base al setpoint
    if (extensorPID_switch){
      outputExtensor = MathUtil.clamp(extensorPID.calculate(posExtensor, setSwitchExtensor), -absClampExtnsr, absClampExtnsr);
    }

    //control manual del extensor; limite touch
    if (control.getTouchpad()){
      extensorPID_switch = false;
      if (control.getPOV() == 90){
        outputExtensor = SmartDashboard.getNumber("Pot. Manual Extensor", 0);
      }
      else if (control.getPOV() == 270){
        outputExtensor = -SmartDashboard.getNumber("Pot. Manual Extensor", 0);
      }
      else{
        outputExtensor = 0;
      }
    }
    else if (control.getPOV() == 90){
      extensorPID_switch = false;
      outputExtensor = SmartDashboard.getNumber("Pot. Manual Extensor", 0);
      if (posExtensor > 50){   //75 cambio 
        outputExtensor = MathUtil.clamp(outputExtensor, -1, 0);
      }
    }
    else if (control.getPOV() == 270){
      extensorPID_switch = false;
      outputExtensor = -SmartDashboard.getNumber("Pot. Manual Extensor", 0);
      if (posExtensor < -0.5){
        outputExtensor = MathUtil.clamp(outputExtensor, 0, 1);
      }
    }
    else if ((control.getPOV() != 90 && control.getPOV() != 270) && !extensorPID_switch){
      outputExtensor = 0;
    }

    // mandar output después de las condiciones
    extensorNeo.set(outputExtensor);

    ////////////////////////////////////////////////////////////////////////////////////////

    // --GARRA-- //

    if (control.getL1Button()){ //le hacemos override si 
      outputGarra = SmartDashboard.getNumber("Garra Subir", 0.0);
    }
    else if(control.getR1Button() && control.getPOV() == -1){
      outputGarra = -SmartDashboard.getNumber("Garra Bajar", 0.0);
    }
    else {//if (operador.getRawButton(7)){
      outputGarra = -1 * operador.getRawAxis(5) * SmartDashboard.getNumber("Garra Subir", 0.0);
    }
    // else{
    //   outputGarra = 0;
    // }

    garraCIM.set(outputGarra);

    // --INTAKE-- //

    if (control.getR2Button() || operador.getRawAxis(3) > 0.25){ // succionar
      potenciaIntake = SmartDashboard.getNumber("Intake IN", 0);
    }
    else if (control.getL2Button() || operador.getRawAxis(2) > 0.25){ //lanzar
      potenciaIntake = -SmartDashboard.getNumber("Intake OUT", 0);
    }
    else {
      potenciaIntake = 0;
    }

    intakeGroup.set(potenciaIntake);

    // --NEUMATICA-- //

    if (control.getCrossButtonPressed() || operador.getRawButtonPressed(1)){ //cerrar
      dSolenoid.set(Value.kForward);
    }
    else if (control.getSquareButtonPressed() || operador.getRawButtonPressed(3)){ //abrir
      dSolenoid.set(Value.kReverse);
    }

    // -- LEDS INFO-- //

    if (ledsCOMP){
      if(operador.getRawButton(6) && operador.getRawButton(5)){
        // sin color
        for (var i = 0; i < INFO_buffer.getLength(); i++) {
          INFO_buffer.setRGB(i, 0, 0, 0);
        }
        INFO_led.setData(INFO_buffer);
      }
      else if (operador.getRawButtonPressed(6)){ //bumper der.
        // morado cubo (61, 0, 184)
        for (var i = 0; i < INFO_buffer.getLength(); i++) {
          INFO_buffer.setRGB(i, 0, 0, 255);
        }
        INFO_led.setData(INFO_buffer);
  
      }
      else if (operador.getRawButtonPressed(5)){ //bumber izq.
        // amarillo cono (225, 100, 0)
        for (var i = 0; i < INFO_buffer.getLength(); i++) {
          INFO_buffer.setRGB(i, 225, 0, 0);
        }
        INFO_led.setData(INFO_buffer);
  
      }
    }

    // --NAVX-- //

    if (control.getOptionsButtonPressed()) { //resetear
      navx.reset();
    }

  }

  @Override
  public void disabledInit() {

    autoDone = false;

    // desactivamos los brakes de todos los motores
    chasisBrake(false);

    extensorNeo.setIdleMode(IdleMode.kCoast);
    garraCIM.setIdleMode(IdleMode.kCoast);
    elevadorNeo.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {
    if (ledsCOMP){
      // arcoíris INFO
      for (var i = 0; i < INFO_buffer.getLength(); i++) {
        final var hue = (INFO_rfph + (i * 180 / INFO_buffer.getLength())) % 180;
        INFO_buffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      INFO_rfph += 4;
      // Check bounds
      INFO_rfph %= 180;

      INFO_led.setData(INFO_buffer);
    }

    SmartDashboard.putBoolean("Control", control.isConnected());
    SmartDashboard.putBoolean("Operador", operador.isConnected());
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  /**
   * Mandar toda la información de la NavX al dashboard.
   */
  private void GyroData() {
    SmartDashboard.putBoolean("Conection", navx.isConnected());
    SmartDashboard.putBoolean("Movimiento", navx.isMoving());
    SmartDashboard.putBoolean("Rotacion", navx.isRotating());
    SmartDashboard.putNumber("Pitch", (navx.getPitch()) );
    SmartDashboard.putNumber("Yaw", navx.getYaw());
    SmartDashboard.putNumber("Yaw", (navx.getYaw()) );
    SmartDashboard.putNumber("Dist. X", (navx.getDisplacementX()) );
    SmartDashboard.putNumber("Dist. Y", (navx.getDisplacementY()) );
    SmartDashboard.putNumber("Vel. X", (navx.getVelocityX()) );
    SmartDashboard.putNumber("Vel. Y", (navx.getVelocityY()) );
  }

  /**
   * Activar y desactivar el brake de los Victor SPX del drivetrain.
   * @param brakeSwitch Al pasar true se activa, al pasar false se desactiva.
   */
  private void chasisBrake(boolean brakeSwitch) {
    if (brakeSwitch) {
      frontLeft.setNeutralMode(NeutralMode.Brake);
      rearLeft.setNeutralMode(NeutralMode.Brake);
      frontRight.setNeutralMode(NeutralMode.Brake);
      rearRight.setNeutralMode(NeutralMode.Brake);
    } else {
      frontLeft.setNeutralMode(NeutralMode.Coast);
      rearLeft.setNeutralMode(NeutralMode.Coast);
      frontRight.setNeutralMode(NeutralMode.Coast);
      rearRight.setNeutralMode(NeutralMode.Coast);
    }
  }
}