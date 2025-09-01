package PowerGAM "Generalized Average Model Library"
  package Examples "Model Cases"
    extends Modelica.Icons.ExamplesPackage;
    package Single_phase_inverter_Examples "One phase full-bridge PWM inverter"
      extends Modelica.Icons.ExamplesPackage;
      model Detail_Model "Detailed Model"
        extends Modelica.Icons.Example;
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto annotation(
          Placement(transformation(origin = {-42, 34}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode annotation(
          Placement(transformation(origin = {-18, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto1 annotation(
          Placement(transformation(origin = {-42, -34}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode1 annotation(
          Placement(transformation(origin = {-18, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto2 annotation(
          Placement(transformation(origin = {62, 34}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode2 annotation(
          Placement(transformation(origin = {86, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto21 annotation(
          Placement(transformation(origin = {62, -34}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode21 annotation(
          Placement(transformation(origin = {86, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) annotation(
          Placement(transformation(origin = {-88, -2}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Electrical.Analog.Basic.Inductor L(L=0.276e-3,i(start=50,fixed=true)) annotation(
          Placement(transformation(origin = {14, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Basic.Resistor RL(R=0.05) annotation(
          Placement(transformation(origin={-14,0}, 
extent={{-10,-10},{10,10}})));
  Modelica.Electrical.Analog.Basic.Capacitor C(C=8e-6) annotation(
          Placement(transformation(origin = {42, -20}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={-116,-50}, 
extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Trapezoid trapezoid(rising = 1 / 20000, width = 0, falling = 1 / 20000, period = 1 / 10000, 
          offset = -1,amplitude=2,startTime=0) annotation(Placement(transformation(origin={-178,-110}, 
extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.GreaterEqual greaterEqual 
          annotation(Placement(transformation(origin={-116,-84}, 
extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Not not1 
          annotation(Placement(transformation(origin={-42,-110}, 
extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.VariableResistor R 
          annotation (Placement(transformation(origin={42,1.11022e-16}, 
extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Step step(startTime=16.7e-3,offset=2,height=3) 
          annotation (Placement(transformation(origin={6,54}, 
extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine(amplitude=0.9,phase(displayUnit="rad")=1,f=60) 
          annotation (Placement(transformation(origin={-178,-66}, 
extent={{-10,-10},{10,10}})));
        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2})),__MWORKS(VisibleVariable={"C.v", 
"L.i", 
"greaterEqual.y"
},ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.1,ContinueTimeVector)),experiment(Algorithm=Dassl,InlineIntegrator=false,InlineStepSize=false,Interval=1e-06,StartTime=0,StopTime=0.1,Tolerance=0.0001));
      equation
        connect(constantVoltage.p, gto.p) annotation(
          Line(points = {{-88, 8}, {-88, 78}, {-30, 78}, {-30, 56}, {-42, 56}, {-42, 44}}, color = {0, 0, 255}));
        connect(diode.n, gto.p) annotation(
          Line(points = {{-18, 44}, {-18, 56}, {-42, 56}, {-42, 44}}, color = {0, 0, 255}));
        connect(gto.n, diode.p) annotation(
          Line(points = {{-42, 24}, {-42, 14}, {-18, 14}, {-18, 24}}, color = {0, 0, 255}));
        connect(gto1.p, diode1.n) annotation(
          Line(points = {{-42, -24}, {-42, -14}, {-18, -14}, {-18, -24}}, color = {0, 0, 255}));
        connect(diode1.p, gto1.n) annotation(
          Line(points = {{-18, -44}, {-18, -60}, {-42, -60}, {-42, -44}}, color = {0, 0, 255}));
        connect(constantVoltage.n, gto1.n) annotation(
          Line(points = {{-88, -12}, {-88, -72}, {-30, -72}, {-30, -60}, {-42, -60}, {-42, -44}}, color = {0, 0, 255}));
        connect(gto.n, gto1.p) annotation(
          Line(points = {{-42, 24}, {-42, 14}, {-30, 14}, {-30, -14}, {-42, -14}, {-42, -24}}, color = {0, 0, 255}));
        connect(gto21.n, constantVoltage.n) annotation(
          Line(points = {{62, -44}, {62, -58}, {74, -58}, {74, -72}, {-88, -72}, {-88, -12}}, color = {0, 0, 255}));
        connect(gto21.p, diode21.n) annotation(
          Line(points = {{62, -24}, {62, -14}, {86, -14}, {86, -24}}, color = {0, 0, 255}));
        connect(gto2.n, diode2.p) annotation(
          Line(points = {{62, 24}, {62, 14}, {86, 14}, {86, 24}}, color = {0, 0, 255}));
        connect(gto2.n, gto21.p) annotation(
          Line(points = {{62, 24}, {62, 14}, {74, 14}, {74, -14}, {62, -14}, {62, -24}}, color = {0, 0, 255}));
  connect(diode21.p, gto21.n) annotation(
          Line(points = {{86, -44}, {86, -58}, {62, -58}, {62, -44}}, color = {0, 0, 255}));
  connect(diode2.n, gto2.p) annotation(
          Line(points = {{86, 44}, {86, 56}, {62, 56}, {62, 44}}, color = {0, 0, 255}));
  connect(gto2.p, gto.p) 
        annotation(Line(origin={15,61}, 
points={{47,-17},{47,-5},{59,-5},{59,17},{-45,17},{-45,-5},{-57,-5},{-57,-17}}, 
color={0,0,255}));
        connect(trapezoid.y, greaterEqual.u2) 
        annotation(Line(origin={-168,-131}, 
points={{0.9999,20.9999},{23.9999,20.9999},{23.9999,39},{39.9999,39}}, 
color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
        connect(greaterEqual.y, gto.fire) 
        annotation(Line(origin={-79,-30}, 
        points={{-26,-54},{9,-54},{9,54},{25,54}}, 
        color={255,0,255}));
        connect(greaterEqual.y, gto21.fire) 
        annotation(Line(origin={-27,-64}, 
points={{-78,-20},{51,-20},{51,20},{77,20}}, 
color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(not1.u, greaterEqual.y) 
        annotation(Line(origin={-79,-97}, 
        points={{25,-13},{-9,-13},{-9,13},{-26,13}}, 
        color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(not1.y, gto1.fire) 
        annotation(Line(origin={-37,-77}, 
        points={{6,-33},{25,-33},{25,13},{-25,13},{-25,33},{-17,33}}, 
        color={255,0,255}));
        connect(not1.y, gto2.fire) 
        annotation(Line(origin={10,-43}, 
        points={{-41,-67},{-10,-67},{-10,67},{40,67}}, 
        color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(step.y, R.R) 
        annotation(Line(origin={7,33}, 
points={{10,21},{21,21},{21,-3},{35,-3},{35,-21}}, 
color={0,0,127}));
        connect(RL.p, gto1.p) 
        annotation(Line(origin={-32,-12}, 
points={{8,12},{2,12},{2,-2},{-10,-2},{-10,-12}}, 
color={0,0,255}));
        connect(RL.n, L.p) 
        annotation(Line(origin={0,0}, 
        points={{-4,0},{4,0}}, 
        color={0,0,255}));
        connect(L.n, R.p) 
        annotation(Line(origin={28,0}, 
        points={{-4,0},{4,1.11022e-16}}, 
        color={0,0,255}));
        connect(R.n, gto21.p) 
        annotation(Line(origin={63,-12}, 
        points={{-11,12},{11,12},{11,-2},{-1,-2},{-1,-12}}, 
        color={0,0,255}));
        connect(C.p, R.p) 
        annotation(Line(origin={30,-10}, 
        points={{2,-10},{-2,-10},{-2,10},{2,10}}, 
        color={0,0,255}));
        connect(C.n, gto21.p) 
        annotation(Line(origin={63,-12}, 
        points={{-11,-8},{-5,-8},{-5,12},{11,12},{11,-2},{-1,-2},{-1,-12}}, 
        color={0,0,255}));
        connect(ground.p, constantVoltage.n) 
        annotation(Line(origin={-102,-42}, 
points={{-14,2},{-14,10},{8,10},{8,-30.511},{14,-30.511},{14,30}}, 
color={0,0,255}));
        connect(cosine.y, greaterEqual.u1) 
        annotation(Line(origin={-147,-75}, 
        points={{-20,9},{-3,9},{-3,-9},{19,-9}}, 
        color={0,0,127}));
      end Detail_Model;
      model Average_Model "Average Model"
        extends Modelica.Icons.Example;
        PowerConverters_GAM.Full_Bridge_Inverter_Ave full_Bridge_Converter_Ave_No_rectifier 
          annotation (Placement(transformation(origin={-13,-6}, 
      extent={{-35,-35},{35,35}})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) annotation(
          Placement(transformation(origin={-85,-6}, 
      extent={{-10,-10},{10,10}}, 
      rotation=-90)));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={-85,-51}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor RL(R=0.05) annotation(
                Placement(transformation(origin={43,15}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Inductor L(L=0.276e-3,i(start=50,fixed=true)) annotation(
                Placement(transformation(origin={74,15}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.VariableResistor R 
          annotation (Placement(transformation(origin={105,15}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Capacitor C(C=8e-6) annotation(
                Placement(transformation(origin={105,49}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Step step(startTime=16.7e-3,offset=2,height=3) 
          annotation (Placement(transformation(origin={53,41}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine(amplitude=0.9,phase(displayUnit="rad")=1,f=60) 
          annotation (Placement(transformation(origin={-64,49}, 
extent={{-10,-10},{10,10}})));
        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
      grid={2,2})),__MWORKS(VisibleVariable={"C.v", 
"L.i"
}));
      equation
        connect(constantVoltage.p, full_Bridge_Converter_Ave_No_rectifier.dc_p) 
        annotation(Line(origin={-66,10}, 
      points={{-19,-6},{-19,5},{18,5}}, 
      color={0,0,255}));
        connect(constantVoltage.n, full_Bridge_Converter_Ave_No_rectifier.dc_n) 
        annotation(Line(origin={-66,-21}, 
      points={{-19,5},{-19,-6},{18,-6}}, 
      color={0,0,255}));
        connect(constantVoltage.n, ground.p) 
        annotation(Line(origin={-85,-28}, 
      points={{0,12},{0,-13}}, 
      color={0,0,255}));
        connect(full_Bridge_Converter_Ave_No_rectifier.ac_p, RL.p) 
        annotation(Line(origin={36,15}, 
      points={{-14,0},{-3,0}}, 
      color={0,0,255}));
        connect(RL.n, L.p) 
        annotation(Line(origin={59,15}, 
      points={{-6,0},{5,0}}, 
      color={0,0,255}));
        connect(L.n, R.p) 
        annotation(Line(origin={90,15}, 
      points={{-6,0},{5,0}}, 
      color={0,0,255}));
        connect(C.p, R.p) 
        annotation(Line(origin={93,32}, 
      points={{2,17},{-2,17},{-2,-17},{2,-17}}, 
      color={0,0,255}));
        connect(C.n, R.n) 
        annotation(Line(origin={118,32}, 
      points={{-3,17},{1,17},{1,-17},{-3,-17}}, 
      color={0,0,255}));
        connect(R.n, full_Bridge_Converter_Ave_No_rectifier.ac_n) 
        annotation(Line(origin={79,-6}, 
      points={{36,21},{56,21},{56,-21},{-57,-21}}, 
      color={0,0,255}));
        connect(step.y, R.R) 
        annotation(Line(origin={85,34}, 
      points={{-21,7},{-6,7},{-6,1},{20,1},{20,-7}}, 
      color={0,0,127}));
        connect(cosine.y, full_Bridge_Converter_Ave_No_rectifier.U_ref) 
        annotation(Line(origin={-33,39}, 
        points={{-20,10},{20,10},{20,-10}}, 
        color={0,0,127}));
        end Average_Model;
      model GAM_Model_Configuration1 "GAM 1"
        extends Modelica.Icons.Example;
        Basic_GAM.Resistor_GAM resistor_GAM(R = 0.05, GAM_Configuration = 2) 
          annotation(Placement(transformation(origin = {46, 20.8}, 
          extent = {{-10, -10}, {10, 10}})));
        Basic_GAM.Inductor_GAM inductor_GAM(L = 0.276e-3, w_sw = 2 * Modelica.Constants.pi * 10000, w = 2 * Modelica.Constants.pi * 60, GAM_Configuration = 2) 
          annotation(Placement(transformation(origin = {76, 20.8}, 
          extent = {{-10, -10}, {10, 10}})));
        Basic_GAM.Capacitor_GAM capacitor_GAM(GAM_Configuration = 2, w_sw = 2 * Modelica.Constants.pi * 10000, w = 2 * Modelica.Constants.pi * 60, C = 8e-6) 
          annotation(Placement(transformation(origin = {110, 50}, 
          extent = {{-10, -10}, {10, 10}})));
        Sensors_GAM.CurrentSensor iL_GAM_Conf1(w_sw = 2 * Modelica.Constants.pi * 10000, w = 2 * Modelica.Constants.pi * 60, GAM_Configuration = 2) 
          annotation(Placement(transformation(origin = {144, 20.8}, 
          extent = {{-10, -10}, {10, 10}})));
        Basic_GAM.Variable_Resistor_GAM variable_Resistor_GAM(GAM_Configuration=2) 
          annotation(Placement(transformation(origin = {110, 20.8}, 
          extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.Step step(startTime = 16.7e-3, offset = 2, height = 3) 
          annotation(Placement(transformation(origin = {56, 64}, 
          extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.Constant m01c(k = 0.486272) 
          annotation(Placement(transformation(origin = {-66, 64}, 
          extent = {{-10, -10}, {10, 10}})));
        Modelica.Blocks.Sources.Constant m01s(k = -0.757324) 
          annotation(Placement(transformation(origin = {-37, 88}, 
          extent = {{-10, -10}, {10, 10}})));
        PowerConverters_GAM.Full_Bridge_GAM full_Bridge_GAM(fai = 0, GAM_Configuration = 2) 
          annotation(Placement(transformation(origin = {-16.5, 2.5}, 
          extent = {{-30.5, -30.5}, {30.5, 30.5}})));
        Sensors_GAM.VoltageSensor vC_GAM_Conf1(GAM_Configuration = 2, w_sw = 2 * Modelica.Constants.pi * 10000, w = 2 * Modelica.Constants.pi * 60) 
          annotation(Placement(transformation(origin = {110, 88}, 
          extent = {{-10, -10}, {10, 10}})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V = 220) 
          annotation(Placement(transformation(origin = {-86, 2.5}, 
          extent = {{-10, -10}, {10, 10}}, 
          rotation = 270)));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation(Placement(transformation(origin = {-86, -38}, 
          extent = {{-10, -10}, {10, 10}})));
        annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, 
          grid = {2, 2})), __MWORKS(VisibleVariable = {"iL_GAM_Conf1.I_GAM", 
"vC_GAM_Conf1.V_GAM"
},ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.1,ContinueTimeVector)),experiment(Algorithm=Dassl,InlineIntegrator=false,InlineStepSize=false,Interval=1e-06,StartTime=0,StopTime=0.1,Tolerance=0.0001));
      equation
        connect(resistor_GAM.plug_n, inductor_GAM.plug_p) 
          annotation(Line(origin = {61, 21}, 
          points = {{-5, -0.2}, {5, -0.2}}, 
          color = {0, 0, 255}));
        connect(inductor_GAM.plug_n, variable_Resistor_GAM.plug_p) 
          annotation(Line(origin = {91, 21}, 
          points = {{-5, -0.2}, {9, -0.2}}, 
          color = {0, 0, 255}));
        connect(iL_GAM_Conf1.plug_p, variable_Resistor_GAM.plug_n) 
          annotation(Line(origin = {125, 21}, 
          points = {{9, -0.2}, {-5, -0.2}}, 
          color = {0, 0, 255}));
        connect(capacitor_GAM.plug_p, variable_Resistor_GAM.plug_p) 
          annotation(Line(origin = {93, 35}, 
          points = {{7, 15}, {-3, 15}, {-3, -14.2}, {7, -14.2}}, 
          color = {0, 0, 255}));
        connect(capacitor_GAM.plug_n, variable_Resistor_GAM.plug_n) 
          annotation(Line(origin = {120, 35}, 
          points = {{0, 15}, {10, 15}, {10, -14.2}, {0, -14.2}}, 
          color = {0, 0, 255}));
        connect(step.y, variable_Resistor_GAM.R) 
          annotation(Line(origin = {89, 48}, 
          points = {{-22, 16}, {-13, 16}, {-13, -10}, {21, -10}, {21, -16.3}}, 
          color = {0, 0, 127}));
        connect(m01c.y, full_Bridge_GAM.m01c) 
          annotation(Line(origin = {-42, 50}, 
          points = {{-13, 14}, {12.08, 14}, {12.08, -13.95}}, 
          color = {0, 0, 127}));
        connect(m01s.y, full_Bridge_GAM.m01s) 
          annotation(Line(origin = {-15, 62}, 
          points = {{-11, 26}, {11.92, 26}, {11.92, -25.95}}, 
          color = {0, 0, 127}));
        connect(full_Bridge_GAM.ac_p, resistor_GAM.plug_p) 
          annotation(Line(origin = {25, 21}, 
          points = {{-11, -0.2}, {11, -0.2}, {11, -0.2}}, 
          color = {0, 0, 255}));
        connect(full_Bridge_GAM.ac_n, iL_GAM_Conf1.plug_n) 
          annotation(Line(origin = {88, 3}, 
          points = {{-74, -18.8}, {74, -18.8}, {74, 17.8}, {66, 17.8}}, 
          color = {0, 0, 255}));
        connect(vC_GAM_Conf1.plug_p, variable_Resistor_GAM.plug_p) 
          annotation(Line(origin = {95, 54}, 
          points = {{5, 34}, {-5, 34}, {-5, -33.2}, {5, -33.2}}, 
          color = {0, 0, 255}));
        connect(vC_GAM_Conf1.plug_n, variable_Resistor_GAM.plug_n) 
          annotation(Line(origin = {125, 54}, 
          points = {{-5, 34}, {5, 34}, {5, -33.2}, {-5, -33.2}}, 
          color = {0, 0, 255}));
        connect(constantVoltage.p, full_Bridge_GAM.dc_p) 
          annotation(Line(origin = {-66, 17}, 
          points = {{-20, -4.5}, {-20, 3.8}, {19, 3.8}}, 
          color = {0, 0, 255}));
        connect(constantVoltage.n, full_Bridge_GAM.dc_n) 
          annotation(Line(origin = {-66, -12}, 
          points = {{-20, 4.5}, {-20, -3.8}, {19, -3.8}}, 
          color = {0, 0, 255}));
        connect(constantVoltage.n, ground.p) 
          annotation(Line(origin = {-86, -18}, 
          points = {{0, 10.5}, {0, -10}}, 
          color = {0, 0, 255}));
      end GAM_Model_Configuration1;
      model GAM_Model_Configuration2 "GAM 2"
        extends Modelica.Icons.Example;
        Basic_GAM.Resistor_GAM resistor_GAM(R=0.05,GAM_Configuration=8) 
          annotation (Placement(transformation(origin={45.5,-1.2}, 
extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=8) 
          annotation (Placement(transformation(origin={75.5,-1.2}, 
extent={{-10,-10},{10,10}})));
        Basic_GAM.Capacitor_GAM capacitor_GAM(GAM_Configuration=8,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,C=8e-6) 
          annotation (Placement(transformation(origin={109.5,28}, 
extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor iL_GAM_Conf2(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=8) 
          annotation (Placement(transformation(origin={143.5,-1.2}, 
extent={{-10,-10},{10,10}})));
        Basic_GAM.Variable_Resistor_GAM variable_Resistor_GAM(GAM_Configuration=8) 
          annotation (Placement(transformation(origin={109.5,-1.2}, 
extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Step step(startTime=16.7e-3,offset=2,height=3) 
          annotation (Placement(transformation(origin={55.5,42}, 
extent={{-10,-10},{10,10}})));
        PowerConverters_GAM.Full_Bridge_GAM full_Bridge_GAM(fai=0,GAM_Configuration=8) 
          annotation (Placement(transformation(origin={-21.5,-19.5}, 
extent={{-30.5,-30.5},{30.5,30.5}})));
        Modelica.Blocks.Sources.Constant m01c(k=0.486272) 
          annotation (Placement(transformation(origin={-76.96,44.975}, 
extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Constant m01s(k=-0.757324) 
          annotation (Placement(transformation(origin={-47.96,68.975}, 
extent={{-10,-10},{10,10}})));
        Sensors_GAM.VoltageSensor vC_GAM_Conf2(GAM_Configuration=8,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60) 
          annotation (Placement(transformation(origin={109.5,58}, 
extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) 
          annotation (Placement(transformation(origin={-120.5,-19.5}, 
extent={{-10,-10},{10,10}}, 
rotation=270)));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={-120.5,-60}, 
extent={{-10,-10},{10,10}})));
        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2})),__MWORKS(VisibleVariable={"iL_GAM_Conf2.I_GAM", 
"iL_GAM_Conf2.currentSensor.i", 
"vC_GAM_Conf2.V_GAM", 
"vC_GAM_Conf2.voltageSensor.v"
},ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.1,ContinueTimeVector)),experiment(Algorithm=Dassl,InlineIntegrator=false,InlineStepSize=false,Interval=1e-06,StartTime=0,StopTime=0.1,Tolerance=0.0001));
      equation
        connect(resistor_GAM.plug_n, inductor_GAM.plug_p) 
        annotation(Line(origin={60.5,-1}, 
points={{-5,-0.2},{5,-0.2}}, 
color={0,0,255}));
        connect(inductor_GAM.plug_n, variable_Resistor_GAM.plug_p) 
        annotation(Line(origin={90.5,-1}, 
points={{-5,-0.2},{9,-0.2}}, 
color={0,0,255}));
        connect(iL_GAM_Conf2.plug_p, variable_Resistor_GAM.plug_n) 
        annotation(Line(origin={124.5,-1}, 
points={{9,-0.2},{-5,-0.2}}, 
color={0,0,255}));
        connect(capacitor_GAM.plug_p, variable_Resistor_GAM.plug_p) 
        annotation(Line(origin={92.5,13}, 
points={{7,15},{-3,15},{-3,-14.2},{7,-14.2}}, 
color={0,0,255}));
        connect(capacitor_GAM.plug_n, variable_Resistor_GAM.plug_n) 
        annotation(Line(origin={119.5,13}, 
points={{0,15},{10,15},{10,-14.2},{0,-14.2}}, 
color={0,0,255}));
        connect(step.y, variable_Resistor_GAM.R) 
        annotation(Line(origin={88.5,26}, 
points={{-22,16},{-13,16},{-13,-10},{21,-10},{21,-16.3}}, 
color={0,0,127}));
        connect(full_Bridge_GAM.ac_p, resistor_GAM.plug_p) 
        annotation(Line(origin={22.5,-1}, 
points={{-13.5,-0.2},{13,-0.2}}, 
color={0,0,255}));
        connect(full_Bridge_GAM.ac_n, iL_GAM_Conf2.plug_n) 
        annotation(Line(origin={86.5,-19}, 
points={{-77.5,-18.8},{77,-18.8},{77,17.8},{67,17.8}}, 
color={0,0,255}));
        connect(m01c.y, full_Bridge_GAM.m01c) 
        annotation(Line(origin={-52.96,30.975}, 
points={{-13,14},{18.04,14},{18.04,-16.925}}, 
color={0,0,127}));
        connect(m01s.y, full_Bridge_GAM.m01s) 
        annotation(Line(origin={-25.96,42.975}, 
points={{-11,26},{17.88,26},{17.88,-28.925}}, 
color={0,0,127}));
        connect(vC_GAM_Conf2.plug_p, variable_Resistor_GAM.plug_p) 
        annotation(Line(origin={94.5,28}, 
points={{5,30},{-5,30},{-5,-29.2},{5,-29.2}}, 
color={0,0,255}));
        connect(vC_GAM_Conf2.plug_n, variable_Resistor_GAM.plug_n) 
        annotation(Line(origin={124.5,28}, 
points={{-5,30},{5,30},{5,-29.2},{-5,-29.2}}, 
color={0,0,255}));
        connect(constantVoltage.p, full_Bridge_GAM.dc_p) 
        annotation(Line(origin={-165,48.6}, 
points={{44.5,-58.1},{44.5,-49.8},{113,-49.8}}, 
color={0,0,255}));
        connect(constantVoltage.n, full_Bridge_GAM.dc_n) 
        annotation(Line(origin={-165,19.6}, 
points={{44.5,-49.1},{44.5,-57.4},{113,-57.4}}, 
color={0,0,255}));
        connect(constantVoltage.n, ground.p) 
        annotation(Line(origin={-120.5,-40}, 
points={{0,10.5},{0,-10}}, 
color={0,0,255}));

      end GAM_Model_Configuration2;
      model GAM_Model_Configuration3 "GAM 3"
        extends Modelica.Icons.Example;
        Basic_GAM.Resistor_GAM resistor_GAM(R=0.05,GAM_Configuration=16) 
          annotation (Placement(transformation(origin={46,20.8}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=16) 
          annotation (Placement(transformation(origin={76,20.8}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Capacitor_GAM capacitor_GAM(GAM_Configuration=16,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,C=8e-6) 
          annotation (Placement(transformation(origin={110,50}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor iL_GAM_Conf4(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=16) 
          annotation (Placement(transformation(origin={144,20.8}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Variable_Resistor_GAM variable_Resistor_GAM(GAM_Configuration=16) 
          annotation (Placement(transformation(origin={110,20.8}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Step step(startTime=16.7e-3,offset=2,height=3) 
          annotation (Placement(transformation(origin={56,64}, 
      extent={{-10,-10},{10,10}})));
        PowerConverters_GAM.Full_Bridge_GAM full_Bridge_GAM(fai=0,GAM_Configuration=16) 
          annotation (Placement(transformation(origin={-21,2.5}, 
      extent={{-30.5,-30.5},{30.5,30.5}})));
        Modelica.Blocks.Sources.Constant m01c(k=0.486272) 
          annotation (Placement(transformation(origin={-76.46,66.975}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Constant m01s(k=-0.757324) 
          annotation (Placement(transformation(origin={-47.46,90.975}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.VoltageSensor vC_GAM_Conf4(GAM_Configuration=16,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60) 
          annotation (Placement(transformation(origin={110,80}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) 
          annotation (Placement(transformation(origin={-120,2.5}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={-120,-38}, 
      extent={{-10,-10},{10,10}})));
        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
      grid={2,2})),__MWORKS(VisibleVariable={"iL_GAM_Conf4.I_GAM", 
"vC_GAM_Conf4.V_GAM"
},ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.1,ContinueTimeVector)),experiment(Algorithm=Dassl,InlineIntegrator=false,InlineStepSize=false,Interval=1e-06,StartTime=0,StopTime=0.1,Tolerance=0.0001));
      equation
        connect(resistor_GAM.plug_n, inductor_GAM.plug_p) 
        annotation(Line(origin={61,21}, 
        points={{-5,-0.2},{5,-0.2}}, 
        color={0,0,255}));
        connect(inductor_GAM.plug_n, variable_Resistor_GAM.plug_p) 
        annotation(Line(origin={91,21}, 
      points={{-5,-0.2},{9,-0.2}}, 
      color={0,0,255}));
        connect(iL_GAM_Conf4.plug_p, variable_Resistor_GAM.plug_n) 
        annotation(Line(origin={125,21}, 
      points={{9,-0.2},{-5,-0.2}}, 
      color={0,0,255}));
        connect(capacitor_GAM.plug_p, variable_Resistor_GAM.plug_p) 
        annotation(Line(origin={93,35}, 
      points={{7,15},{-3,15},{-3,-14.2},{7,-14.2}}, 
      color={0,0,255}));
        connect(capacitor_GAM.plug_n, variable_Resistor_GAM.plug_n) 
        annotation(Line(origin={120,35}, 
      points={{0,15},{10,15},{10,-14.2},{0,-14.2}}, 
      color={0,0,255}));
        connect(step.y, variable_Resistor_GAM.R) 
        annotation(Line(origin={89,48}, 
        points={{-22,16},{-13,16},{-13,-10},{21,-10},{21,-16.3}}, 
        color={0,0,127}));
        connect(full_Bridge_GAM.ac_p, resistor_GAM.plug_p) 
        annotation(Line(origin={23,21}, 
        points={{-13.5,-0.2},{13,-0.2},{13,-0.2}}, 
        color={0,0,255}));
        connect(full_Bridge_GAM.ac_n, iL_GAM_Conf4.plug_n) 
        annotation(Line(origin={87,3}, 
        points={{-77.5,-18.8},{77,-18.8},{77,17.8},{67,17.8}}, 
        color={0,0,255}));
        connect(m01c.y, full_Bridge_GAM.m01c) 
        annotation(Line(origin={-52.46,52.975}, 
        points={{-13,14},{12.08,14},{12.08,-13.95}}, 
        color={0,0,127}));
        connect(m01s.y, full_Bridge_GAM.m01s) 
        annotation(Line(origin={-25.46,64.975}, 
        points={{-11,26},{11.92,26},{11.92,-25.95}}, 
        color={0,0,127}));
        connect(vC_GAM_Conf4.plug_p, variable_Resistor_GAM.plug_p) 
        annotation(Line(origin={95,50}, 
        points={{5,30},{-5,30},{-5,-29.2},{5,-29.2}}, 
        color={0,0,255}));
        connect(vC_GAM_Conf4.plug_n, variable_Resistor_GAM.plug_n) 
        annotation(Line(origin={125,50}, 
        points={{-5,30},{5,30},{5,-29.2},{-5,-29.2}}, 
        color={0,0,255}));
        connect(constantVoltage.p, full_Bridge_GAM.dc_p) 
        annotation(Line(origin={-164.5,70.6}, 
      points={{44.5,-58.1},{44.5,-49.8},{113,-49.8}}, 
      color={0,0,255}));
        connect(constantVoltage.n, full_Bridge_GAM.dc_n) 
        annotation(Line(origin={-164.5,41.6}, 
      points={{44.5,-49.1},{44.5,-57.4},{113,-57.4}}, 
      color={0,0,255}));
        connect(constantVoltage.n, ground.p) 
        annotation(Line(origin={-120,-18}, 
      points={{0,10.5},{0,-10}}, 
      color={0,0,255}));

      end GAM_Model_Configuration3;
      end Single_phase_inverter_Examples;
    package Three_phase_inverter_Examples "Three phase PWM inverter"
      extends Modelica.Icons.ExamplesPackage;
      model Detail_Model "Detailed Model"
        extends Modelica.Icons.Example;
        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
      grid={2,2})),__MWORKS(VisibleVariable={"La.i", 
"Lb.i", 
"Lc.i", 
"greaterEqual.y"
},ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.005,ContinueTimeVector)),experiment(Algorithm=Dassl,Interval=1e-06,StartTime=0,StopTime=0.1,Tolerance=1e-06,InlineIntegrator=false,InlineStepSize=false));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto annotation(
          Placement(transformation(origin={-68,28}, 
      extent={{10,-10},{-10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode annotation(
          Placement(transformation(origin={-44,28}, 
      extent={{-10,-10},{10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto1 annotation(
          Placement(transformation(origin={-68,-30}, 
      extent={{10,-10},{-10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode1 annotation(
          Placement(transformation(origin={-44,-30}, 
      extent={{-10,-10},{10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto2 annotation(
          Placement(transformation(origin={-20,28}, 
      extent={{10,-10},{-10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode2 annotation(
          Placement(transformation(origin={4,28}, 
      extent={{-10,-10},{10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto3 annotation(
          Placement(transformation(origin={-20,-30}, 
      extent={{10,-10},{-10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode3 annotation(
          Placement(transformation(origin={4,-30}, 
      extent={{-10,-10},{10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto4 annotation(
          Placement(transformation(origin={28,28}, 
      extent={{10,-10},{-10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode4 annotation(
          Placement(transformation(origin={52,28}, 
      extent={{-10,-10},{10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealGTOThyristor gto5 annotation(
          Placement(transformation(origin={28,-30}, 
      extent={{10,-10},{-10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Ideal.IdealDiode diode5 annotation(
          Placement(transformation(origin={52,-30}, 
      extent={{-10,-10},{10,10}}, 
      rotation=90)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) annotation(
          Placement(transformation(origin={-108,-2}, 
      extent={{-10,-10},{10,10}}, 
      rotation=-90)));
        Modelica.Electrical.Analog.Basic.Resistor RLa(R=0.05) annotation(
                Placement(transformation(origin={84,18}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor RLb(R=0.05) annotation(
                Placement(transformation(origin={84,2.22045e-16}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor RLc(R=0.05) annotation(
                Placement(transformation(origin={84,-18}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Inductor La(L=0.276e-3) 
          annotation (Placement(transformation(origin={116,18}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Inductor Lb(L=0.276e-3) 
          annotation (Placement(transformation(origin={116,0}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Inductor Lc(L=0.276e-3) 
          annotation (Placement(transformation(origin={116,-18}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={196,-18}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Trapezoid trapezoid(rising = 1 / 20000, width = 0, falling = 1 / 20000, period = 1 / 10000, 
          offset = -1,amplitude=2,startTime=0) annotation(Placement(transformation(origin={-220,-164}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.GreaterEqual greaterEqual 
          annotation(Placement(transformation(origin={-158,-138}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine(amplitude=0.911,phase(displayUnit="rad")=0.0441,f=60) 
          annotation (Placement(transformation(origin={-260,-84}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine1(amplitude=0.875,phase(displayUnit="rad")=0.0561,f=60) 
          annotation (Placement(transformation(origin={-260,-126}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Switch switch1 
          annotation (Placement(transformation(origin={-206,-106}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=time < 16.7e-3) 
          annotation (Placement(transformation(origin={-312,-106}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Not not1 
          annotation (Placement(transformation(origin={-108,-164}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.GreaterEqual greaterEqual1 
          annotation(Placement(transformation(origin={-20,-222}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Trapezoid trapezoid1(rising = 1 / 20000, width = 0, falling = 1 / 20000, period = 1 / 10000, 
          offset = -1,amplitude=2,startTime=0) annotation(Placement(transformation(origin={-68,-248}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine2(amplitude=0.911,phase(displayUnit="rad")=0.0441+120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={-152,-200}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine3(amplitude=0.875,phase(displayUnit="rad")=0.0561+120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={-152,-242}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Switch switch2 
          annotation (Placement(transformation(origin={-98,-222}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=time < 16.7e-3) 
          annotation (Placement(transformation(origin={-204,-222}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Not not2 
          annotation (Placement(transformation(origin={28,-248}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Trapezoid trapezoid2(rising = 1 / 20000, width = 0, falling = 1 / 20000, period = 1 / 10000, 
          offset = -1,amplitude=2,startTime=0) annotation(Placement(transformation(origin={84,-370}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine4(amplitude=0.911,phase(displayUnit="rad")=0.0441-120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={0,-322}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine5(amplitude=0.875,phase(displayUnit="rad")=0.0561-120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={0,-364}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Switch switch3 
          annotation (Placement(transformation(origin={54,-344}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.BooleanExpression booleanExpression2(y=time < 16.7e-3) 
          annotation (Placement(transformation(origin={-52,-344}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.GreaterEqual greaterEqual2 
          annotation(Placement(transformation(origin={132,-344}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Not not3 
          annotation (Placement(transformation(origin={178,-370}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor R1(R=2.2) annotation(
                Placement(transformation(origin={150,18}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor R2(R=2.2) annotation(
                Placement(transformation(origin={150,2.22045e-16}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor R3(R=2.2) annotation(
                Placement(transformation(origin={150,-18}, 
      extent={{-10,-10},{10,10}})));
        equation
        connect(diode.n, gto.p) 
        annotation(Line(origin={-56,44}, 
      points={{12,-6},{12,6},{-12,6},{-12,-6}}, 
      color={0,0,255}));
        connect(gto.n, diode.p) 
        annotation(Line(origin={-56,13}, 
      points={{-12,5},{-12,-5},{12,-5},{12,5}}, 
      color={0,0,255}));
        connect(diode1.n, gto1.p) 
        annotation(Line(origin={-56,-14}, 
      points={{12,-6},{12,6},{-12,6},{-12,-6}}, 
      color={0,0,255}));
        connect(gto1.n, diode1.p) 
        annotation(Line(origin={-56,-45}, 
      points={{-12,5},{-12,-5},{12,-5},{12,5}}, 
      color={0,0,255}));
        connect(diode2.n, gto2.p) 
        annotation(Line(origin={-8,44}, 
      points={{12,-6},{12,6},{-12,6},{-12,-6}}, 
      color={0,0,255}));
        connect(gto2.n, diode2.p) 
        annotation(Line(origin={-8,13}, 
      points={{-12,5},{-12,-5},{12,-5},{12,5}}, 
      color={0,0,255}));
        connect(diode3.n, gto3.p) 
        annotation(Line(origin={-8,-14}, 
      points={{12,-6},{12,6},{-12,6},{-12,-6}}, 
      color={0,0,255}));
        connect(gto3.n, diode3.p) 
        annotation(Line(origin={-8,-45}, 
      points={{-12,5},{-12,-5},{12,-5},{12,5}}, 
      color={0,0,255}));
        connect(diode4.n, gto4.p) 
        annotation(Line(origin={40,44}, 
      points={{12,-6},{12,6},{-12,6},{-12,-6}}, 
      color={0,0,255}));
        connect(gto4.n, diode4.p) 
        annotation(Line(origin={40,13}, 
      points={{-12,5},{-12,-5},{12,-5},{12,5}}, 
      color={0,0,255}));
        connect(diode5.n, gto5.p) 
        annotation(Line(origin={40,-14}, 
      points={{12,-6},{12,6},{-12,6},{-12,-6}}, 
      color={0,0,255}));
        connect(gto5.n, diode5.p) 
        annotation(Line(origin={40,-45}, 
      points={{-12,5},{-12,-5},{12,-5},{12,5}}, 
      color={0,0,255}));
        connect(constantVoltage.p, gto.p) 
        annotation(Line(origin={-84,40}, 
      points={{-24,-32},{-24,32},{28,32},{28,10},{16,10},{16,-2}}, 
      color={0,0,255}));
        connect(constantVoltage.p, gto2.p) 
        annotation(Line(origin={-60,40}, 
      points={{-48,-32},{-48,32},{52,32},{52,10},{40,10},{40,-2}}, 
      color={0,0,255}));
        connect(constantVoltage.p, gto4.p) 
        annotation(Line(origin={-35,40}, 
      points={{-73,-32},{-73,32},{75,32},{75,10},{63,10},{63,-2}}, 
      color={0,0,255}));
        connect(gto.n, gto1.p) 
        annotation(Line(origin={-62,-1}, 
      points={{-6,19},{-6,9},{6,9},{6,-7},{-6,-7},{-6,-19}}, 
      color={0,0,255}));
        connect(gto2.n, gto3.p) 
        annotation(Line(origin={-14,-1}, 
      points={{-6,19},{-6,9},{6,9},{6,-7},{-6,-7},{-6,-19}}, 
      color={0,0,255}));
        connect(gto4.n, gto5.p) 
        annotation(Line(origin={34,-1}, 
      points={{-6,19},{-6,9},{6,9},{6,-7},{-6,-7},{-6,-19}}, 
      color={0,0,255}));
        connect(constantVoltage.n, diode5.p) 
        annotation(Line(origin={-24,-43}, 
      points={{-84,31},{-84,-31},{64,-31},{64,-7},{76,-7},{76,3}}, 
      color={0,0,255}));
        connect(gto1.n, diode5.p) 
        annotation(Line(origin={-8,-57}, 
      points={{-60,17},{-60,7},{-48,7},{-48,-17},{48,-17},{48,7},{60,7},{60,17}}, 
      color={0,0,255}));
        connect(gto3.n, diode5.p) 
        annotation(Line(origin={16,-57}, 
      points={{-36,17},{-36,7},{-24,7},{-24,-17},{24,-17},{24,7},{36,7},{36,17}}, 
      color={0,0,255}));
        connect(RLa.p, gto1.p) 
        annotation(Line(origin={3,0}, 
      points={{71,18},{55,18},{55,4},{-59,4},{-59,-8},{-71,-8},{-71,-20}}, 
      color={0,0,255}));
        connect(RLb.p, gto3.p) 
        annotation(Line(origin={27,-9}, 
      points={{47,9},{-35,9},{-35,1},{-47,1},{-47,-11}}, 
      color={0,0,255}));
        connect(RLc.p, gto5.p) 
        annotation(Line(origin={51,-12}, 
      points={{23,-6},{7,-6},{7,8},{-11,8},{-11,4},{-23,4},{-23,-8}}, 
      color={0,0,255}));
        connect(RLa.n, La.p) 
        annotation(Line(origin={100,18}, 
        points={{-6,0},{6,0}}, 
        color={0,0,255}));
        connect(RLb.n, Lb.p) 
        annotation(Line(origin={100,0}, 
        points={{-6,2.22045e-16},{6,2.22045e-16},{6,0}}, 
        color={0,0,255}));
        connect(RLc.n, Lc.p) 
        annotation(Line(origin={100,-18}, 
        points={{-6,0},{6,0}}, 
        color={0,0,255}));
        connect(trapezoid.y, greaterEqual.u2) 
        annotation(Line(origin={-210,-185}, 
      points={{1,21},{23.9999,21},{23.9999,39},{40,39}}, 
      color={0,0,127}));
        connect(switch1.u1, cosine.y) 
        annotation(Line(origin={-233,-91}, 
      points={{15,-7},{-5,-7},{-5,7},{-16,7}}, 
      color={0,0,127}));
        connect(switch1.u3, cosine1.y) 
        annotation(Line(origin={-233,-120}, 
      points={{15,6},{-5,6},{-5,-6},{-16,-6}}, 
      color={0,0,127}));
        connect(switch1.y, greaterEqual.u1) 
        annotation(Line(origin={-182,-122}, 
        points={{-13,16},{-4,16},{-4,-16},{12,-16}}, 
        color={0,0,127}));
        connect(booleanExpression.y, switch1.u2) 
        annotation(Line(origin={-259,-106}, 
        points={{-42,0},{41,0}}, 
        color={255,0,255}));
        connect(greaterEqual.y, gto.fire) 
        annotation(Line(origin={-113,-60}, 
      points={{-34,-78},{23,-78},{23,78},{33,78}}, 
      color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(not1.u, greaterEqual.y) 
        annotation(Line(origin={-121,-148}, 
      points={{1,-16},{-9,-16},{-9,10},{-26,10}}, 
      color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(not1.y, gto1.fire) 
        annotation(Line(origin={-75,-99}, 
      points={{-22,-65},{-11,-65},{-11,59},{-5,59}}, 
      color={255,0,255}));
        connect(trapezoid1.y, greaterEqual1.u2) 
        annotation(Line(origin={-44,-239}, 
      points={{-13,-9},{4,-9},{4,9},{12,9}}, 
      color={0,0,127}));
        connect(switch2.u1, cosine2.y) 
        annotation(Line(origin={-125,-207}, 
      points={{15,-7},{-5,-7},{-5,7},{-16,7}}, 
      color={0,0,127}));
        connect(switch2.u3, cosine3.y) 
        annotation(Line(origin={-125,-236}, 
      points={{15,6},{-5,6},{-5,-6},{-16,-6}}, 
      color={0,0,127}));
        connect(booleanExpression1.y, switch2.u2) 
        annotation(Line(origin={-151,-222}, 
      points={{-42,0},{41,0}}, 
      color={255,0,255}));
        connect(switch2.y, greaterEqual1.u1) 
        annotation(Line(origin={-59,-223}, 
      points={{-28,1},{27,1}}, 
      color={0,0,127}));
        connect(greaterEqual1.y, gto2.fire) 
        annotation(Line(origin={-12,-102}, 
        points={{3,-120},{28,-120},{28,-38},{-28,-38},{-28,120},{-20,120}}, 
        color={255,0,255}));
        connect(not2.u, greaterEqual1.y) 
        annotation(Line(origin={4,-235}, 
        points={{12,-13},{-4,-13},{-4,13},{-13,13}}, 
        color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(not2.y, gto3.fire) 
        annotation(Line(origin={2,-144}, 
      points={{37,-104},{46,-104},{46,14},{-38,14},{-38,104},{-34,104}}, 
      color={255,0,255}));
        connect(switch3.u1, cosine4.y) 
        annotation(Line(origin={27,-329}, 
      points={{15,-7},{-5,-7},{-5,7},{-16,7}}, 
      color={0,0,127}));
        connect(switch3.u3, cosine5.y) 
        annotation(Line(origin={27,-358}, 
      points={{15,6},{-5,6},{-5,-6},{-16,-6}}, 
      color={0,0,127}));
        connect(booleanExpression2.y, switch3.u2) 
        annotation(Line(origin={1,-344}, 
      points={{-42,0},{41,0}}, 
      color={255,0,255}));
        connect(switch3.y, greaterEqual2.u1) 
        annotation(Line(origin={93,-344}, 
        points={{-28,0},{27,0}}, 
        color={0,0,127}));
        connect(trapezoid2.y, greaterEqual2.u2) 
        annotation(Line(origin={108,-361}, 
        points={{-13,-9},{2,-9},{2,9},{12,9}}, 
        color={0,0,127}));
        connect(greaterEqual2.y, gto4.fire) 
        annotation(Line(origin={87,-163}, 
      points={{56,-181},{79,-181},{79,53},{-77,53},{-77,181},{-71,181}}, 
      color={255,0,255}));
        connect(not3.u, greaterEqual2.y) 
        annotation(Line(origin={155,-357}, 
        points={{11,-13},{-3,-13},{-3,13},{-12,13}}, 
        color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(not3.y, gto5.fire) 
        annotation(Line(origin={106,-205}, 
      points={{83,-165},{90,-165},{90,115},{-94,115},{-94,165},{-90,165}}, 
      color={255,0,255}),__MWORKS(BlockSystem(NamedSignal)));
        connect(La.n, R1.p) 
        annotation(Line(origin={133,18}, 
        points={{-7,0},{7,0}}, 
        color={0,0,255}));
        connect(Lb.n, R2.p) 
        annotation(Line(origin={133,0}, 
        points={{-7,0},{7,0},{7,2.22045e-16}}, 
        color={0,0,255}));
        connect(Lc.n, R3.p) 
        annotation(Line(origin={133,-18}, 
        points={{-7,0},{7,0}}, 
        color={0,0,255}));
        connect(R1.n, ground.p) 
        annotation(Line(origin={178,5}, 
        points={{-18,13},{-2,13},{-2,-5},{18,-5},{18,-13}}, 
        color={0,0,255}));
        connect(R2.n, ground.p) 
        annotation(Line(origin={178,-4}, 
        points={{-18,4},{18,4},{18,-4}}, 
        color={0,0,255}));
        connect(R3.n, ground.p) 
        annotation(Line(origin={178,-9}, 
        points={{-18,-9},{-2,-9},{-2,9},{18,9},{18,1}}, 
        color={0,0,255}));
        end Detail_Model;
      model Average_Model "Average Model"
        extends Modelica.Icons.Example;
        PowerConverters_GAM.Three_Phase_2_Level_Inverter_Ave three_Phase_2_Level_Converter_Ave_No_rectifier 
          annotation (Placement(transformation(origin={-2,-2}, 
      extent={{-45,-45},{45,45}})));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) annotation(
          Placement(transformation(origin={-92,-2}, 
      extent={{-10,-10},{10,10}}, 
      rotation=-90)));
        Modelica.Blocks.Sources.Cosine cosine(amplitude=0.911,phase(displayUnit="rad")=0.0441,f=60) 
          annotation (Placement(transformation(origin={-222,156}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine1(amplitude=0.875,phase(displayUnit="rad")=0.0561,f=60) 
          annotation (Placement(transformation(origin={-222,114}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Switch switch1 
          annotation (Placement(transformation(origin={-168,134}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.BooleanExpression booleanExpression(y=time < 16.7e-3) 
          annotation (Placement(transformation(origin={-274,134}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine2(amplitude=0.911,phase(displayUnit="rad")=0.0441+120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={-169,238}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine3(amplitude=0.875,phase(displayUnit="rad")=0.0561+120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={-169,196}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Switch switch2 
          annotation (Placement(transformation(origin={-115,216}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.BooleanExpression booleanExpression1(y=time < 16.7e-3) 
          annotation (Placement(transformation(origin={-221,216}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine4(amplitude=0.911,phase(displayUnit="rad")=0.0441-120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={120,234}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Cosine cosine5(amplitude=0.875,phase(displayUnit="rad")=0.0561-120*Modelica.Constants.pi/180,f=60) 
          annotation (Placement(transformation(origin={120,192}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Logical.Switch switch3 
          annotation (Placement(transformation(origin={174,212}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.BooleanExpression booleanExpression2(y=time < 16.7e-3) 
          annotation (Placement(transformation(origin={68,212}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor RLa(R=0.05) annotation(
                Placement(transformation(origin={76,16}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor RLb(R=0.05) annotation(
                Placement(transformation(origin={76,-2}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor RLc(R=0.05) annotation(
                Placement(transformation(origin={76,-20}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Inductor La(L=0.276e-3) 
          annotation (Placement(transformation(origin={108,16}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Inductor Lb(L=0.276e-3) 
          annotation (Placement(transformation(origin={108,-2}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Inductor Lc(L=0.276e-3) 
          annotation (Placement(transformation(origin={108,-20}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={188,-20}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor R1(R=2.2) annotation(
                Placement(transformation(origin={142,16}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor R2(R=2.2) annotation(
                Placement(transformation(origin={142,-2}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Resistor R3(R=2.2) annotation(
                Placement(transformation(origin={142,-20}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Electrical.Analog.Basic.Ground ground1 
          annotation (Placement(transformation(origin={-92,-54}, 
      extent={{-10,-10},{10,10}})));
        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
      grid={2,2})),__MWORKS(VisibleVariable={"La.i", 
      "Lb.i", 
      "Lc.i"
      },ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.005,ContinueTimeVector)),experiment(Algorithm=Dassl,Interval=1e-06,StartTime=0,StopTime=0.1,Tolerance=1e-06,InlineIntegrator=false,InlineStepSize=false));
      equation
        connect(constantVoltage.p, three_Phase_2_Level_Converter_Ave_No_rectifier.dc_p) 
        annotation(Line(origin={-69,17}, 
        points={{-23,-9},{-23,8},{22,8}}, 
        color={0,0,255}));
        connect(constantVoltage.n, three_Phase_2_Level_Converter_Ave_No_rectifier.dc_n) 
        annotation(Line(origin={-69,-20}, 
        points={{-23,8},{-23,-9},{22,-9}}, 
        color={0,0,255}));
        connect(switch1.u1, cosine.y) 
        annotation(Line(origin={-195,149}, 
      points={{15,-7},{-5,-7},{-5,7},{-16,7}}, 
      color={0,0,127}));
        connect(switch1.u3, cosine1.y) 
        annotation(Line(origin={-195,120}, 
      points={{15,6},{-5,6},{-5,-6},{-16,-6}}, 
      color={0,0,127}));
        connect(booleanExpression.y, switch1.u2) 
        annotation(Line(origin={-221,134}, 
      points={{-42,0},{41,0}}, 
      color={255,0,255}));
        connect(switch2.u1, cosine2.y) 
        annotation(Line(origin={-142,231}, 
      points={{15,-7},{-5,-7},{-5,7},{-16,7}}, 
      color={0,0,127}));
        connect(switch2.u3, cosine3.y) 
        annotation(Line(origin={-142,202}, 
      points={{15,6},{-5,6},{-5,-6},{-16,-6}}, 
      color={0,0,127}));
        connect(booleanExpression1.y, switch2.u2) 
        annotation(Line(origin={-168,216}, 
      points={{-42,0},{41,0}}, 
      color={255,0,255}));
        connect(switch3.u1, cosine4.y) 
        annotation(Line(origin={147,227}, 
        points={{15,-7},{-5,-7},{-5,7},{-16,7}}, 
        color={0,0,127}));
        connect(switch3.u3, cosine5.y) 
        annotation(Line(origin={147,198}, 
        points={{15,6},{-5,6},{-5,-6},{-16,-6}}, 
        color={0,0,127}));
        connect(booleanExpression2.y, switch3.u2) 
        annotation(Line(origin={121,212}, 
        points={{-42,0},{41,0}}, 
        color={255,0,255}));
        connect(RLa.n, La.p) 
        annotation(Line(origin={92,16}, 
      points={{-6,0},{6,0}}, 
      color={0,0,255}));
        connect(RLb.n, Lb.p) 
        annotation(Line(origin={92,-2}, 
      points={{-6,0},{6,0}}, 
      color={0,0,255}));
        connect(RLc.n, Lc.p) 
        annotation(Line(origin={92,-20}, 
      points={{-6,0},{6,0}}, 
      color={0,0,255}));
        connect(La.n, R1.p) 
        annotation(Line(origin={125,16}, 
      points={{-7,0},{7,0}}, 
      color={0,0,255}));
        connect(Lb.n, R2.p) 
        annotation(Line(origin={125,-2}, 
      points={{-7,0},{7,0}}, 
      color={0,0,255}));
        connect(Lc.n, R3.p) 
        annotation(Line(origin={125,-20}, 
      points={{-7,0},{7,0}}, 
      color={0,0,255}));
        connect(R1.n, ground.p) 
        annotation(Line(origin={170,3}, 
      points={{-18,13},{-2,13},{-2,-5},{18,-5},{18,-13}}, 
      color={0,0,255}));
        connect(R2.n, ground.p) 
        annotation(Line(origin={170,-6}, 
      points={{-18,4},{18,4},{18,-4}}, 
      color={0,0,255}));
        connect(R3.n, ground.p) 
        annotation(Line(origin={170,-11}, 
      points={{-18,-9},{-2,-9},{-2,9},{18,9},{18,1}}, 
      color={0,0,255}));
        connect(RLb.p, three_Phase_2_Level_Converter_Ave_No_rectifier.ac_b) 
        annotation(Line(origin={55,-2}, 
        points={{11,0},{-12,0}}, 
        color={0,0,255}));
        connect(RLc.p, three_Phase_2_Level_Converter_Ave_No_rectifier.ac_c) 
        annotation(Line(origin={55,-24}, 
        points={{11,4},{-1,4},{-1,-4.1},{-12,-4.1}}, 
        color={0,0,255}));
        connect(RLa.p, three_Phase_2_Level_Converter_Ave_No_rectifier.ac_a) 
        annotation(Line(origin={55,20}, 
      points={{11,-4},{-1,-4},{-1,4.1},{-12,4.1}}, 
      color={0,0,255}));
        connect(constantVoltage.n, ground1.p) 
        annotation(Line(origin={-92,-28}, 
        points={{0,16},{0,-16}}, 
        color={0,0,255}));
        connect(switch1.y, three_Phase_2_Level_Converter_Ave_No_rectifier.Ua_ref) 
        annotation(Line(origin={-93,89}, 
        points={{-64,45},{63.325,45},{63.325,-46}}, 
        color={0,0,127}));
        connect(switch2.y, three_Phase_2_Level_Converter_Ave_No_rectifier.Ub_ref) 
        annotation(Line(origin={-53,130}, 
        points={{-51,86},{51,86},{51,-87}}, 
        color={0,0,127}));
        connect(switch3.y, three_Phase_2_Level_Converter_Ave_No_rectifier.Uc_ref) 
        annotation(Line(origin={122,128}, 
        points={{63,84},{98,84},{98,-18},{-97.225,-18},{-97.225,-85}}, 
        color={0,0,127}));

      end Average_Model;
      model GAM_Model_Configuration1 "GAM 1"
        extends Modelica.Icons.Example;
      PowerConverters_GAM.Three_two_level_Converter_GAM three_two_level_Converter_GAM(GAM_Configuration=2,fai=0) 
          annotation (Placement(transformation(origin={-2,2}, 
      extent={{-42,-42},{42,42}})));
        Basic_GAM.Resistor_GAM resistor_GAM(R=0.05,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={71,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={102,27.2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={164,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM1(R=2.2,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={133,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM2(R=0.05,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={71,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM1(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={102,2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor1(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={164,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM3(R=2.2,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={133,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM4(R=0.05,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={71,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM2(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={102,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor2(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={164,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM5(R=2.2,GAM_Configuration=2) 
          annotation (Placement(transformation(origin={133,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Step ma01c(startTime=16.7e-3,offset=0.911*cos(0.0441),height=0.875*cos(0.0561)-0.911*cos(0.0441)) 
          annotation (Placement(transformation(origin={-84,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s(startTime=16.7e-3,offset=-0.911*sin(0.0441),height=-0.875*sin(0.0561)+0.911*sin(0.0441)) 
          annotation (Placement(transformation(origin={-47.88,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01c1(startTime=16.7e-3,offset=0.911*cos(0.0441+(2/3)*Modelica.Constants.pi),height=0.875*cos(0.0561+(2/3)*Modelica.Constants.pi)-0.911*cos(0.0441+(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={-19.64,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s1(startTime=16.7e-3,offset=-0.911*sin(0.0441+(2/3)*Modelica.Constants.pi),height=-0.875*sin(0.0561+(2/3)*Modelica.Constants.pi)+0.911*sin(0.0441+(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={15.64,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01c2(startTime=16.7e-3,offset=0.911*cos(0.0441-(2/3)*Modelica.Constants.pi),height=0.875*cos(0.0561-(2/3)*Modelica.Constants.pi)-0.911*cos(0.0441-(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={46,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s2(startTime=16.7e-3,offset=-0.911*sin(0.0441-(2/3)*Modelica.Constants.pi),height=-0.875*sin(0.0561-(2/3)*Modelica.Constants.pi)+0.911*sin(0.0441-(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={82,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) 
          annotation (Placement(transformation(origin={-84,2}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={-84,-56}, 
      extent={{-10,-10},{10,10}})));









        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2})),__MWORKS(VisibleVariable={"currentSensor.I_GAM", 
"currentSensor1.I_GAM", 
"currentSensor2.I_GAM"
},ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.0025,ContinueTimeVector)),experiment(Algorithm=Dassl,Interval=1e-06,StartTime=0,StopTime=0.05,Tolerance=1e-06));
      equation
        connect(three_two_level_Converter_GAM.ac_pa, resistor_GAM.plug_p) 
        annotation(Line(origin={52,27}, 
      points={{-12,0.2},{9,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM.plug_p, resistor_GAM.plug_n) 
        annotation(Line(origin={92,27}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM.plug_n, resistor_GAM1.plug_p) 
        annotation(Line(origin={118,27}, 
        points={{-6,0.2},{5,0.2}}, 
        color={0,0,255}));
        connect(resistor_GAM1.plug_n, currentSensor.plug_p) 
        annotation(Line(origin={149,27}, 
        points={{-6,0.2},{5,0.2},{5,0.2}}, 
        color={0,0,255}));
        connect(inductor_GAM1.plug_p, resistor_GAM2.plug_n) 
        annotation(Line(origin={92,1.8}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM1.plug_n, resistor_GAM3.plug_p) 
        annotation(Line(origin={118,1.8}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(resistor_GAM3.plug_n, currentSensor1.plug_p) 
        annotation(Line(origin={149,1.8}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM2.plug_p, resistor_GAM4.plug_n) 
        annotation(Line(origin={92,-23.4}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM2.plug_n, resistor_GAM5.plug_p) 
        annotation(Line(origin={118,-23.4}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(resistor_GAM5.plug_n, currentSensor2.plug_p) 
        annotation(Line(origin={149,-23.4}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(three_two_level_Converter_GAM.ac_pb, resistor_GAM2.plug_p) 
        annotation(Line(origin={51,2}, 
        points={{-11,0},{10,0}}, 
        color={0,0,255}));
        connect(three_two_level_Converter_GAM.ac_pc, resistor_GAM4.plug_p) 
        annotation(Line(origin={51,-23}, 
        points={{-11,-0.2},{10,-0.2}}, 
        color={0,0,255}));
        connect(currentSensor.plug_n, currentSensor2.plug_n) 
        annotation(Line(origin={179,2}, 
        points={{-5,25.2},{5,25.2},{5,-25.2},{-5,-25.2}}, 
        color={0,0,255}));
        connect(currentSensor1.plug_n, currentSensor2.plug_n) 
        annotation(Line(origin={179,-11}, 
        points={{-5,13},{5,13},{5,-12.2},{-5,-12.2}}, 
        color={0,0,255}));
        connect(ma01c.y, three_two_level_Converter_GAM.ma01c) 
        annotation(Line(origin={-50,69}, 
      points={{-34,20},{-34,-7},{18.6,-7},{18.6,-20.8}}, 
      color={0,0,127}));
        connect(ma01s.y, three_two_level_Converter_GAM.ma01s) 
        annotation(Line(origin={-27,69}, 
      points={{-20.88,20},{-20.88,-1},{7.36,-1},{7.36,-20.8}}, 
      color={0,0,127}));
        connect(ma01c1.y, three_two_level_Converter_GAM.mb01c) 
        annotation(Line(origin={-14,69}, 
        points={{-5.64,20},{-5.64,5},{6.12,5},{6.12,-20.8}}, 
        color={0,0,127}));
        connect(ma01s1.y, three_two_level_Converter_GAM.mb01s) 
        annotation(Line(origin={10,69}, 
        points={{5.64,20},{5.64,5},{-6.12,5},{-6.12,-20.8}}, 
        color={0,0,127}));
        connect(ma01c2.y, three_two_level_Converter_GAM.mc01c) 
        annotation(Line(origin={31,69}, 
        points={{15,20},{15,-1},{-15.36,-1},{-15.36,-20.8}}, 
        color={0,0,127}));
        connect(ma01s2.y, three_two_level_Converter_GAM.mc01s) 
        annotation(Line(origin={55,69}, 
      points={{27,20},{27,-7},{-27.6,-7},{-27.6,-20.8}}, 
      color={0,0,127}));
        connect(constantVoltage.p, three_two_level_Converter_GAM.dc_p) 
        annotation(Line(origin={-86,16.5}, 
      points={{2,-4.5},{2,10.7},{42,10.7}}, 
      color={0,0,255}));
        connect(constantVoltage.n, three_two_level_Converter_GAM.dc_n) 
        annotation(Line(origin={-86,-12.5}, 
      points={{2,4.5},{2,-10.7},{42,-10.7}}, 
      color={0,0,255}));
        connect(constantVoltage.n, ground.p) 
        annotation(Line(origin={-84,-18.5}, 
      points={{0,10.5},{0,-27.5}}, 
      color={0,0,255}));
        end GAM_Model_Configuration1;
      model GAM_Model_Configuration2 "GAM 2"
        extends Modelica.Icons.Example;
      PowerConverters_GAM.Three_two_level_Converter_GAM three_two_level_Converter_GAM(GAM_Configuration=6,fai=0) 
          annotation (Placement(transformation(origin={-2,2}, 
      extent={{-42,-42},{42,42}})));
        Basic_GAM.Resistor_GAM resistor_GAM(R=0.05,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={71,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={102,27.2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={164,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM1(R=2.2,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={133,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM2(R=0.05,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={71,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM1(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={102,2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor1(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={164,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM3(R=2.2,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={133,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM4(R=0.05,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={71,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM2(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={102,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor2(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={164,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM5(R=2.2,GAM_Configuration=6) 
          annotation (Placement(transformation(origin={133,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Step ma01c(startTime=16.7e-3,offset=0.911*cos(0.0441),height=0.875*cos(0.0561)-0.911*cos(0.0441)) 
          annotation (Placement(transformation(origin={-84,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s(startTime=16.7e-3,offset=-0.911*sin(0.0441),height=-0.875*sin(0.0561)+0.911*sin(0.0441)) 
          annotation (Placement(transformation(origin={-47.88,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01c1(startTime=16.7e-3,offset=0.911*cos(0.0441+(2/3)*Modelica.Constants.pi),height=0.875*cos(0.0561+(2/3)*Modelica.Constants.pi)-0.911*cos(0.0441+(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={-19.64,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s1(startTime=16.7e-3,offset=-0.911*sin(0.0441+(2/3)*Modelica.Constants.pi),height=-0.875*sin(0.0561+(2/3)*Modelica.Constants.pi)+0.911*sin(0.0441+(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={15.64,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01c2(startTime=16.7e-3,offset=0.911*cos(0.0441-(2/3)*Modelica.Constants.pi),height=0.875*cos(0.0561-(2/3)*Modelica.Constants.pi)-0.911*cos(0.0441-(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={46,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s2(startTime=16.7e-3,offset=-0.911*sin(0.0441-(2/3)*Modelica.Constants.pi),height=-0.875*sin(0.0561-(2/3)*Modelica.Constants.pi)+0.911*sin(0.0441-(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={82,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) 
          annotation (Placement(transformation(origin={-84,2}, 
extent={{-10,-10},{10,10}}, 
rotation=270)));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={-84,-56}, 
extent={{-10,-10},{10,10}})));









        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2})),__MWORKS(VisibleVariable={"currentSensor.I_GAM", 
"currentSensor1.I_GAM", 
"currentSensor2.I_GAM"
},ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.0025,ContinueTimeVector)),experiment(Algorithm=Dassl,Interval=1e-06,StartTime=0,StopTime=0.05,Tolerance=1e-06));
      equation
        connect(three_two_level_Converter_GAM.ac_pa, resistor_GAM.plug_p) 
        annotation(Line(origin={52,27}, 
      points={{-12,0.2},{9,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM.plug_p, resistor_GAM.plug_n) 
        annotation(Line(origin={92,27}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM.plug_n, resistor_GAM1.plug_p) 
        annotation(Line(origin={118,27}, 
        points={{-6,0.2},{5,0.2}}, 
        color={0,0,255}));
        connect(resistor_GAM1.plug_n, currentSensor.plug_p) 
        annotation(Line(origin={149,27}, 
        points={{-6,0.2},{5,0.2},{5,0.2}}, 
        color={0,0,255}));
        connect(inductor_GAM1.plug_p, resistor_GAM2.plug_n) 
        annotation(Line(origin={92,1.8}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM1.plug_n, resistor_GAM3.plug_p) 
        annotation(Line(origin={118,1.8}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(resistor_GAM3.plug_n, currentSensor1.plug_p) 
        annotation(Line(origin={149,1.8}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM2.plug_p, resistor_GAM4.plug_n) 
        annotation(Line(origin={92,-23.4}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM2.plug_n, resistor_GAM5.plug_p) 
        annotation(Line(origin={118,-23.4}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(resistor_GAM5.plug_n, currentSensor2.plug_p) 
        annotation(Line(origin={149,-23.4}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(three_two_level_Converter_GAM.ac_pb, resistor_GAM2.plug_p) 
        annotation(Line(origin={51,2}, 
        points={{-11,0},{10,0}}, 
        color={0,0,255}));
        connect(three_two_level_Converter_GAM.ac_pc, resistor_GAM4.plug_p) 
        annotation(Line(origin={51,-23}, 
        points={{-11,-0.2},{10,-0.2}}, 
        color={0,0,255}));
        connect(currentSensor.plug_n, currentSensor2.plug_n) 
        annotation(Line(origin={179,2}, 
        points={{-5,25.2},{5,25.2},{5,-25.2},{-5,-25.2}}, 
        color={0,0,255}));
        connect(currentSensor1.plug_n, currentSensor2.plug_n) 
        annotation(Line(origin={179,-11}, 
        points={{-5,13},{5,13},{5,-12.2},{-5,-12.2}}, 
        color={0,0,255}));
        connect(ma01c.y, three_two_level_Converter_GAM.ma01c) 
        annotation(Line(origin={-50,69}, 
      points={{-34,20},{-34,-7},{18.6,-7},{18.6,-20.8}}, 
      color={0,0,127}));
        connect(ma01s.y, three_two_level_Converter_GAM.ma01s) 
        annotation(Line(origin={-27,69}, 
      points={{-20.88,20},{-20.88,-1},{7.36,-1},{7.36,-20.8}}, 
      color={0,0,127}));
        connect(ma01c1.y, three_two_level_Converter_GAM.mb01c) 
        annotation(Line(origin={-14,69}, 
        points={{-5.64,20},{-5.64,5},{6.12,5},{6.12,-20.8}}, 
        color={0,0,127}));
        connect(ma01s1.y, three_two_level_Converter_GAM.mb01s) 
        annotation(Line(origin={10,69}, 
        points={{5.64,20},{5.64,5},{-6.12,5},{-6.12,-20.8}}, 
        color={0,0,127}));
        connect(ma01c2.y, three_two_level_Converter_GAM.mc01c) 
        annotation(Line(origin={31,69}, 
        points={{15,20},{15,-1},{-15.36,-1},{-15.36,-20.8}}, 
        color={0,0,127}));
        connect(ma01s2.y, three_two_level_Converter_GAM.mc01s) 
        annotation(Line(origin={55,69}, 
      points={{27,20},{27,-7},{-27.6,-7},{-27.6,-20.8}}, 
      color={0,0,127}));
        connect(constantVoltage.p, three_two_level_Converter_GAM.dc_p) 
        annotation(Line(origin={-86,16.5}, 
points={{2,-4.5},{2,10.7},{42,10.7}}, 
color={0,0,255}));
        connect(constantVoltage.n, three_two_level_Converter_GAM.dc_n) 
        annotation(Line(origin={-86,-12.5}, 
points={{2,4.5},{2,-10.7},{42,-10.7}}, 
color={0,0,255}));
        connect(constantVoltage.n, ground.p) 
        annotation(Line(origin={-84,-18.5}, 
points={{0,10.5},{0,-27.5}}, 
color={0,0,255}));
        end GAM_Model_Configuration2;
      model GAM_Model_Configuration3 "GAM 3"
        extends Modelica.Icons.Example;
      PowerConverters_GAM.Three_two_level_Converter_GAM three_two_level_Converter_GAM(GAM_Configuration=14,fai=0) 
          annotation (Placement(transformation(origin={-2,2}, 
      extent={{-42,-42},{42,42}})));
        Basic_GAM.Resistor_GAM resistor_GAM(R=0.05,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={71,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={102,27.2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={164,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM1(R=2.2,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={133,27.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM2(R=0.05,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={71,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM1(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={102,2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor1(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={164,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM3(R=2.2,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={133,2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM4(R=0.05,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={71,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Inductor_GAM inductor_GAM2(L=0.276e-3,w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={102,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Sensors_GAM.CurrentSensor currentSensor2(w_sw=2*Modelica.Constants.pi*10000,w=2*Modelica.Constants.pi*60,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={164,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Basic_GAM.Resistor_GAM resistor_GAM5(R=2.2,GAM_Configuration=14) 
          annotation (Placement(transformation(origin={133,-23.2}, 
      extent={{-10,-10},{10,10}})));
        Modelica.Blocks.Sources.Step ma01c(startTime=16.7e-3,offset=0.911*cos(0.0441),height=0.875*cos(0.0561)-0.911*cos(0.0441)) 
          annotation (Placement(transformation(origin={-84,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s(startTime=16.7e-3,offset=-0.911*sin(0.0441),height=-0.875*sin(0.0561)+0.911*sin(0.0441)) 
          annotation (Placement(transformation(origin={-47.88,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01c1(startTime=16.7e-3,offset=0.911*cos(0.0441+(2/3)*Modelica.Constants.pi),height=0.875*cos(0.0561+(2/3)*Modelica.Constants.pi)-0.911*cos(0.0441+(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={-19.64,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s1(startTime=16.7e-3,offset=-0.911*sin(0.0441+(2/3)*Modelica.Constants.pi),height=-0.875*sin(0.0561+(2/3)*Modelica.Constants.pi)+0.911*sin(0.0441+(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={15.64,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01c2(startTime=16.7e-3,offset=0.911*cos(0.0441-(2/3)*Modelica.Constants.pi),height=0.875*cos(0.0561-(2/3)*Modelica.Constants.pi)-0.911*cos(0.0441-(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={46,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Blocks.Sources.Step ma01s2(startTime=16.7e-3,offset=-0.911*sin(0.0441-(2/3)*Modelica.Constants.pi),height=-0.875*sin(0.0561-(2/3)*Modelica.Constants.pi)+0.911*sin(0.0441-(2/3)*Modelica.Constants.pi)) 
          annotation (Placement(transformation(origin={82,100}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)));
        Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=220) 
          annotation (Placement(transformation(origin={-98,2}, 
extent={{-10,-10},{10,10}}, 
rotation=270)));
        Modelica.Electrical.Analog.Basic.Ground ground 
          annotation (Placement(transformation(origin={-98,-68}, 
extent={{-10,-10},{10,10}})));









        annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2})),__MWORKS(ContinueSimConfig(SaveContinueFile="false",SaveBeforeStop="false",NumberBeforeStop=1,FixedContinueInterval="false",ContinueIntervalLength=0.005,ContinueTimeVector),VisibleVariable={"currentSensor.I_GAM", 
"currentSensor1.I_GAM", 
"currentSensor2.I_GAM"
}),experiment(Algorithm=Dassl,Interval=1e-06,StartTime=0,StopTime=0.1,Tolerance=1e-06,InlineIntegrator=false,InlineStepSize=false));
      equation
        connect(three_two_level_Converter_GAM.ac_pa, resistor_GAM.plug_p) 
        annotation(Line(origin={52,27}, 
      points={{-12,0.2},{9,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM.plug_p, resistor_GAM.plug_n) 
        annotation(Line(origin={92,27}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM.plug_n, resistor_GAM1.plug_p) 
        annotation(Line(origin={118,27}, 
        points={{-6,0.2},{5,0.2}}, 
        color={0,0,255}));
        connect(resistor_GAM1.plug_n, currentSensor.plug_p) 
        annotation(Line(origin={149,27}, 
        points={{-6,0.2},{5,0.2},{5,0.2}}, 
        color={0,0,255}));
        connect(inductor_GAM1.plug_p, resistor_GAM2.plug_n) 
        annotation(Line(origin={92,1.8}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM1.plug_n, resistor_GAM3.plug_p) 
        annotation(Line(origin={118,1.8}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(resistor_GAM3.plug_n, currentSensor1.plug_p) 
        annotation(Line(origin={149,1.8}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM2.plug_p, resistor_GAM4.plug_n) 
        annotation(Line(origin={92,-23.4}, 
      points={{0,0.2},{-11,0.2}}, 
      color={0,0,255}));
        connect(inductor_GAM2.plug_n, resistor_GAM5.plug_p) 
        annotation(Line(origin={118,-23.4}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(resistor_GAM5.plug_n, currentSensor2.plug_p) 
        annotation(Line(origin={149,-23.4}, 
      points={{-6,0.2},{5,0.2}}, 
      color={0,0,255}));
        connect(three_two_level_Converter_GAM.ac_pb, resistor_GAM2.plug_p) 
        annotation(Line(origin={51,2}, 
        points={{-11,0},{10,0}}, 
        color={0,0,255}));
        connect(three_two_level_Converter_GAM.ac_pc, resistor_GAM4.plug_p) 
        annotation(Line(origin={51,-23}, 
        points={{-11,-0.2},{10,-0.2}}, 
        color={0,0,255}));
        connect(currentSensor.plug_n, currentSensor2.plug_n) 
        annotation(Line(origin={179,2}, 
        points={{-5,25.2},{5,25.2},{5,-25.2},{-5,-25.2}}, 
        color={0,0,255}));
        connect(currentSensor1.plug_n, currentSensor2.plug_n) 
        annotation(Line(origin={179,-11}, 
        points={{-5,13},{5,13},{5,-12.2},{-5,-12.2}}, 
        color={0,0,255}));
        connect(ma01c.y, three_two_level_Converter_GAM.ma01c) 
        annotation(Line(origin={-50,69}, 
      points={{-34,20},{-34,-7},{18.6,-7},{18.6,-20.8}}, 
      color={0,0,127}));
        connect(ma01s.y, three_two_level_Converter_GAM.ma01s) 
        annotation(Line(origin={-27,69}, 
      points={{-20.88,20},{-20.88,-1},{7.36,-1},{7.36,-20.8}}, 
      color={0,0,127}));
        connect(ma01c1.y, three_two_level_Converter_GAM.mb01c) 
        annotation(Line(origin={-14,69}, 
        points={{-5.64,20},{-5.64,5},{6.12,5},{6.12,-20.8}}, 
        color={0,0,127}));
        connect(ma01s1.y, three_two_level_Converter_GAM.mb01s) 
        annotation(Line(origin={10,69}, 
        points={{5.64,20},{5.64,5},{-6.12,5},{-6.12,-20.8}}, 
        color={0,0,127}));
        connect(ma01c2.y, three_two_level_Converter_GAM.mc01c) 
        annotation(Line(origin={31,69}, 
        points={{15,20},{15,-1},{-15.36,-1},{-15.36,-20.8}}, 
        color={0,0,127}));
        connect(ma01s2.y, three_two_level_Converter_GAM.mc01s) 
        annotation(Line(origin={55,69}, 
      points={{27,20},{27,-7},{-27.6,-7},{-27.6,-20.8}}, 
      color={0,0,127}));
        connect(constantVoltage.p, three_two_level_Converter_GAM.dc_p) 
        annotation(Line(origin={-117,5.9}, 
points={{19,6.1},{19,21.3},{73,21.3}}, 
color={0,0,255}));
        connect(constantVoltage.n, three_two_level_Converter_GAM.dc_n) 
        annotation(Line(origin={-117,-23.1}, 
points={{19,15.1},{19,-0.1},{73,-0.1}}, 
color={0,0,255}));
        connect(constantVoltage.n, ground.p) 
        annotation(Line(origin={-115,-29.1}, 
points={{17,21.1},{17,-28.9}}, 
color={0,0,255}));
        end GAM_Model_Configuration3;
      end Three_phase_inverter_Examples;
    end Examples;

  package Basic_GAM "Passive Component Model Library"
    annotation(
      Icon(graphics = {Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(origin = {11.626, 40}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-80, -70}, {60, -10}}), Line(origin = {11.626, 40}, points = {{60, -40}, {80, -40}}, color = {0, 0, 255}), Line(points = {{-88.374, 0}, {-68.374, 0}}, color = {0, 0, 255})}));
    model Ground_GAM "Ground node"
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      PowerGAM.Interfaces_GAM.Plug_GAM p(GAM_Configuration=GAM_Configuration) annotation(Placement(transformation(origin={0,96}, 
    extent={{10,-10},{-10,10}}, 
    rotation=270)));
    equation
      p.pin.v = zeros(GAM_Configuration);
      annotation(
        Documentation(info = "<html>
<p>Ground of an electrical circuit. The potential at the ground node is zero. Every electrical circuit has to contain at least one ground object.</p>
</html>"                    , 
        revisions = "<html>
<ul>
<li><em> 1998   </em>
by Christoph Clauss<br> initially implemented<br>
</li>
</ul>
</html>"                    ), 
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 
        100}}), graphics = {
        Line(points = {{-60, 50}, {60, 50}}, color = {0, 0, 255}), 
        Line(points = {{-40, 30}, {40, 30}}, color = {0, 0, 255}), 
        Line(points = {{-20, 10}, {20, 10}}, color = {0, 0, 255}), 
        Line(points = {{0, 90}, {0, 50}}, color = {0, 0, 255}), 
        Text(
        extent = {{-150, -10}, {150, -50}}, 
        textString = "%name", 
        textColor = {0, 0, 255})}));
    end Ground_GAM;
    model Resistor_GAM "GAM of Resistor"
      extends PowerGAM.Interfaces_GAM.OnePort_GAM;
      parameter Modelica.Units.SI.Resistance R(start=1) "电阻值";
      annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Rectangle(origin={7.10543e-15,0}, 
lineColor={0,0,255}, 
fillColor={255,255,255}, 
fillPattern=FillPattern.Solid, 
extent={{-70,30},{70,-30}}), Line(origin={-80,0}, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Line(origin={80,0}, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Text(origin={1,78}, 
lineColor={0,0,0}, 
extent={{-111,30},{111,-30}}, 
textString="Order  = %GAM_Configuration", 
textStyle={TextStyle.None}, 
textColor={0,0,0}, 
horizontalAlignment=LinePattern.None), Text(origin={7.10543e-15,-68}, 
lineColor={0,0,255}, 
extent={{-78,26},{78,-26}}, 
textString="%name", 
fontName="Times New Roman", 
textStyle={TextStyle.None}, 
textColor={0,0,255})}));
    equation
      v = R * i;
    end Resistor_GAM;
    model Inductor_GAM "GAM of Inductor"
      extends PowerGAM.Interfaces_GAM.OnePort_GAM;
      parameter Real w_sw "开关频率";
      parameter Real w "系统频率";
      parameter Modelica.Units.SI.Inductance L(start=1) "电感值";
      Real Ohm[GAM_Configuration, GAM_Configuration];
      annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Line(origin={75,0}, 
points={{-15,0},{15,0}}, 
color={0,0,255}), Line(origin={-75,0}, 
points={{-15,0},{15,0}}, 
color={0,0,255}), Line(origin={-45,7}, 
points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, 
color={0,0,255}, 
smooth=Smooth.Bezier), Line(origin={-15,7}, 
points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, 
color={0,0,255}, 
smooth=Smooth.Bezier), Line(origin={15,7}, 
points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, 
color={0,0,255}, 
smooth=Smooth.Bezier), Line(origin={45,7}, 
points={{-15,-7},{-14,-1},{-7,7},{7,7},{14,-1},{15,-7}}, 
color={0,0,255}, 
smooth=Smooth.Bezier), Text(origin={2,-54}, 
lineColor={0,0,255}, 
extent={{-78,26},{78,-26}}, 
textString="%name", 
fontName="Times New Roman", 
textStyle={TextStyle.None}, 
textColor={0,0,255}), Text(origin={2,70}, 
lineColor={0,0,0}, 
extent={{-100,26},{100,-26}}, 
textString="Order  = %GAM_Configuration", 
textStyle={TextStyle.None}, 
textColor={0,0,0}, 
horizontalAlignment=LinePattern.None)}));
    equation
      L * (-Ohm * i + der(i)) = v;
      Ohm = Functions.Func_Ohm(GAM_Configuration,w_sw,w);
    end Inductor_GAM;
    model Capacitor_GAM "GAM of Capacitor"
      extends PowerGAM.Interfaces_GAM.OnePort_GAM;
      parameter Real w_sw "开关频率";
      parameter Real w "系统频率";
      parameter Modelica.Units.SI.Capacitance C(start=1) "电容值";
      Real Ohm[GAM_Configuration, GAM_Configuration];
      annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Line(origin={-6,0}, 
points={{0,28},{0,-28}}, 
color={0,0,255}), Line(origin={6,0}, 
points={{0,28},{0,-28}}, 
color={0,0,255}), Line(origin={-48,0}, 
points={{-42,0},{42,0}}, 
color={0,0,255}), Line(origin={48,0}, 
points={{-42,0},{42,0}}, 
color={0,0,255}), Text(origin={-3.55271e-15,68}, 
lineColor={0,0,0}, 
extent={{-99,32},{99,-32}}, 
textString="Order = %GAM_Configuration", 
textStyle={TextStyle.None}, 
textColor={0,0,0}, 
horizontalAlignment=LinePattern.None), Text(origin={-1.77636e-15,-62}, 
lineColor={0,0,255}, 
extent={{-78,26},{78,-26}}, 
textString="%name", 
fontName="Times New Roman", 
textStyle={TextStyle.None}, 
textColor={0,0,255})}));
    equation
      C * (-Ohm * v + der(v)) = i;
      Ohm = Functions.Func_Ohm(GAM_Configuration,w_sw,w);
    end Capacitor_GAM;
    model Variable_Resistor_GAM "GAM of Variable Resistor"
      extends PowerGAM.Interfaces_GAM.OnePort_GAM;
      Modelica.Blocks.Interfaces.RealInput R(unit="Ohm") 
        annotation (Placement(transformation(origin={0,109}, 
    extent={{-9,-9},{9,9}}, 
    rotation=270)));
      annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Rectangle(origin={7.10543e-15,0}, 
lineColor={0,0,255}, 
fillColor={255,255,255}, 
fillPattern=FillPattern.Solid, 
extent={{-70,30},{70,-30}}), Line(origin={-80,0}, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Line(origin={80,0}, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Text(origin={-1,68}, 
lineColor={0,0,0}, 
extent={{-99,30},{99,-30}}, 
textString="Order  = %GAM_Configuration", 
textStyle={TextStyle.None}, 
textColor={0,0,0}, 
horizontalAlignment=LinePattern.None), Text(origin={7.10543e-15,-68}, 
lineColor={0,0,255}, 
extent={{-78,26},{78,-26}}, 
textString="%name", 
fontName="Times New Roman", 
textStyle={TextStyle.None}, 
textColor={0,0,255})}));
    equation
      v = R * i;
    end Variable_Resistor_GAM;
  end Basic_GAM;
  package Sources_GAM "Source Component Model Library"
    annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Rectangle(origin={0,0}, 
lineColor={200,200,200}, 
fillColor={248,248,248}, 
fillPattern=FillPattern.HorizontalCylinder, 
extent={{-100,-100},{100,100}}, 
radius=25), Rectangle(origin={0,0}, 
lineColor={128,128,128}, 
extent={{-100,-100},{100,100}}, 
radius=25), Polygon(origin={23.3333,0}, 
fillColor={128,128,128}, 
pattern=LinePattern.None, 
fillPattern=FillPattern.Solid, 
points={{-23.333,30},{46.667,0},{-23.333,-30}}), Rectangle(origin={-35,0}, 
fillColor={128,128,128}, 
pattern=LinePattern.None, 
fillPattern=FillPattern.Solid, 
extent={{-35,-4.5},{35,4.5}})}));
    model SignalVoltage_GAM "GAM of Signal Voltage Source"
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      Modelica.Units.SI.Current i[GAM_Configuration]=plug_p.pin.i 
        "Currents flowing into positive plugs";
      PowerGAM.Interfaces_GAM.PositivePlug_GAM plug_p(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{-110,-10},{-90,10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM plug_n(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealInput v[GAM_Configuration](each unit="V") 
        "Voltage between pin p and n (= p.v - n.v) as input signal" annotation (
         Placement(transformation(
            origin={0,120}, 
            extent={{-20,-20},{20,20}}, 
            rotation=270)));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage[GAM_Configuration] 
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation
      connect(signalVoltage.p, plug_p.pin) 
        annotation (Line(points={{-10,0},{-100,0}}, color={0,0,255}));
      connect(signalVoltage.n, plug_n.pin) 
        annotation (Line(points={{10,0},{100,0}}, color={0,0,255}));
      connect(v, signalVoltage.v) 
        annotation (Line(points={{0,120},{0,12}}, color={0,0,255}));
      annotation (
        Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Line(origin={-70,0}, 
    points={{-20,0},{20,0}}, 
    color={0,0,255}), Line(origin={70,0}, 
    points={{-20,0},{20,0}}, 
    color={0,0,255}), Ellipse(origin={0,0}, 
    lineColor={0,0,255}, 
    fillColor={255,255,255}, 
    fillPattern=FillPattern.Solid, 
    extent={{-50,50},{50,-50}}), Line(origin={0,0}, 
    points={{-50,0},{50,0}}, 
    color={0,0,255}), Line(origin={-70,20}, 
    points={{0,10},{0,-10}}, 
    color={0,0,255}), Line(origin={-70,20}, 
    points={{-10,0},{10,0}}, 
    color={0,0,255}), Line(origin={70,20}, 
    points={{-10,0},{10,0}}, 
    color={0,0,255}), Text(origin={-1.06581e-14,183}, 
    extent={{150,-15},{-150,15}}, 
    textString="GAM 阶数=%GAM_Configuration"), Text(origin={0,-76}, 
    lineColor={0,0,255}, 
    extent={{-78,26},{78,-26}}, 
    textString="%name", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,255})}), 
                                     Documentation(info="<html>
<p>
Contains m signal controlled voltage sources (Modelica.Electrical.Analog.Sources.SignalVoltage)
</p>
</html>"                    ));
    end SignalVoltage_GAM;
    model SignalCurrent_GAM "GAM of Signal Current Source"
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      Modelica.Units.SI.Voltage v[GAM_Configuration]=plug_p.pin.v - plug_n.pin.v 
        "Voltage drops between the two plugs";
      PowerGAM.Interfaces_GAM.PositivePlug_GAM plug_p(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{-110,-10},{-90,10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM plug_n(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealInput i[GAM_Configuration](each unit="A") 
        "Current flowing from pin p to pin n as input signal" annotation (
          Placement(transformation(
            origin={0,120}, 
            extent={{-20,-20},{20,20}}, 
            rotation=270)));
      Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent[GAM_Configuration] 
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation
      connect(signalCurrent.p, plug_p.pin) 
        annotation (Line(points={{-10,0},{-100,0}}, color={0,0,255}));
      connect(signalCurrent.n, plug_n.pin) 
        annotation (Line(points={{10,0},{100,0}}, color={0,0,255}));
      connect(i, signalCurrent.i) 
        annotation (Line(points={{0,120},{0,12}}, color={0,0,255}));
      annotation (
        Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Line(origin={-70,0}, 
    points={{-20,0},{20,0}}, 
    color={0,0,255}), Line(origin={70,0}, 
    points={{-20,0},{20,0}}, 
    color={0,0,255}), Ellipse(origin={0,0}, 
    lineColor={0,0,255}, 
    fillColor={255,255,255}, 
    fillPattern=FillPattern.Solid, 
    extent={{-50,50},{50,-50}}), Line(origin={0,0}, 
    points={{0,50},{0,-50}}, 
    color={0,0,255}), Polygon(origin={75,0}, 
    lineColor={0,0,255}, 
    fillColor={0,0,255}, 
    fillPattern=FillPattern.Solid, 
    points={{15,0},{-15,10},{-15,-10},{15,0}}), Text(origin={-6,185}, 
    extent={{150,-15},{-150,15}}, 
    textString="GAM 阶数=%GAM_Configuration"), Text(origin={7.10543e-15,-82}, 
    lineColor={0,0,255}, 
    extent={{-78,26},{78,-26}}, 
    textString="%name", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,255})}), Documentation(info="<html>
<p>
Contains m signal controlled current sources (Modelica.Electrical.Analog.Sources.SignalCurrent)
</p>
</html>"                            ));
    end SignalCurrent_GAM;

  end Sources_GAM;
  package PowerConverters_GAM "Inverter Model Library"
    annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Rectangle(origin={0,0}, 
lineColor={200,200,200}, 
fillColor={248,248,248}, 
fillPattern=FillPattern.HorizontalCylinder, 
extent={{-100,-100},{100,100}}, 
radius=25), Rectangle(origin={0,0}, 
lineColor={128,128,128}, 
extent={{-100,-100},{100,100}}, 
radius=25), Line(origin={1,0}, 
points={{-79,0},{79,0}}, 
color={95,95,95}), Line(origin={36,-1}, 
points={{0,51},{0,-51}}, 
color={95,95,95}), Polygon(origin={1,0}, 
lineColor={95,95,95}, 
points={{35,0},{-35,50},{-35,-50},{35,0}})}));
    model Full_Bridge_GAM "GAM of Full-Bridge PWM Inverter"
      extends Interfaces_GAM.DCACfourpin_GAM;
      parameter Real fai "载波信号的相位/rad";


      Real[GAM_Configuration,1] q;

      Real V0;
      Real[GAM_Configuration] Vac;
      Real[GAM_Configuration] Iac;
      Real[1] I0;

      Sources_GAM.SignalVoltage_GAM signalVoltage_GAM(GAM_Configuration=GAM_Configuration) 
        annotation (Placement(transformation(origin={18,7.77156e-16}, 
    extent={{10,-10},{-10,10}}, 
    rotation=90)));
      Basic_GAM.Resistor_GAM resistor_GAM(R=0.0001,GAM_Configuration=GAM_Configuration) 
        annotation (Placement(transformation(origin={44,60}, 
    extent={{-10,-10},{10,10}})));
      Sensors_GAM.CurrentSensor_GAM currentSensor_GAM(GAM_Configuration=GAM_Configuration) 
        annotation (Placement(transformation(origin={76,60}, 
    extent={{-10,-10},{10,10}}, 
    rotation=360)));
      Modelica.Blocks.Interfaces.RealInput m01c "调制波信号的余弦项系数" 
        annotation (Placement(transformation(origin={-44,110}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)));
      Modelica.Blocks.Interfaces.RealInput m01s "调制波信号的正弦项系数" 
        annotation (Placement(transformation(origin={44,110}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)));
      Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent 
        annotation (Placement(transformation(origin={-78,0}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor 
        annotation (Placement(transformation(origin={-36,8.88178e-16}, 
    extent={{10,-10},{-10,10}}, 
    rotation=-270)));
      Basic_GAM.Ground_GAM ground_GAM(GAM_Configuration=GAM_Configuration) 
        annotation (Placement(transformation(origin={-8.88178e-16,-74}, 
extent={{-10,-10},{10,10}})));
      annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2})),Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Rectangle(origin={0,0}, 
lineColor={0,0,0}, 
fillColor={255,202,216}, 
fillPattern=FillPattern.Solid, 
extent={{-100,100},{100,-100}}), Text(origin={0,-92}, 
lineColor={0,0,0}, 
extent={{-62.5,10},{62.5,-10}}, 
textString="Order  = %GAM_Configuration", 
textStyle={TextStyle.None}, 
textColor={0,0,0}, 
horizontalAlignment=LinePattern.None), Line(origin={-48,42}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={-56,42}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={-62,42}, 
rotation=360, 
points={{-6,0},{6,0}}, 
color={255,0,255}), Line(origin={-38,64}, 
rotation=360, 
points={{-10,-18},{10,2},{10,18}}, 
color={0,0,255}), Line(origin={-38,20}, 
rotation=360, 
points={{-10,18},{10,-2},{10,-18}}, 
color={0,0,255}), Line(origin={-38,28}, 
rotation=360, 
points={{6,-6},{0,6},{-6,0},{6,-6}}, 
color={0,0,255}), Line(origin={-23,42}, 
rotation=360, 
points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
color={0,0,255}), Line(origin={-18,50}, 
rotation=360, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Line(origin={-18,42}, 
rotation=360, 
points={{0,8},{-10,-8},{10,-8},{0,8}}, 
color={0,0,255}), Line(origin={-48,-38}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={-56,-38}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={-62,-38}, 
rotation=360, 
points={{-6,0},{6,0}}, 
color={255,0,255}), Line(origin={-38,-16}, 
rotation=360, 
points={{-10,-18},{10,2},{10,18}}, 
color={0,0,255}), Line(origin={-38,-60}, 
rotation=360, 
points={{-10,18},{10,-2},{10,-18}}, 
color={0,0,255}), Line(origin={-38,-52}, 
rotation=360, 
points={{6,-6},{0,6},{-6,0},{6,-6}}, 
color={0,0,255}), Line(origin={-23,-38}, 
rotation=360, 
points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
color={0,0,255}), Line(origin={-18,-30}, 
rotation=360, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Line(origin={-18,-38}, 
rotation=360, 
points={{0,8},{-10,-8},{10,-8},{0,8}}, 
color={0,0,255}), Line(origin={20,10}, 
rotation=360, 
points={{48,0},{-48,0}}, 
color={0,0,255}), Line(origin={-15,82}, 
rotation=360, 
points={{61,0},{-61,0}}, 
color={0,0,255}), Line(origin={-16,-78}, 
rotation=360, 
points={{62,0},{-62,0}}, 
color={0,0,255}), Ellipse(origin={-78,-78}, 
rotation=360, 
lineColor={0,0,255}, 
extent={{-2,2},{2,-2}}), Ellipse(origin={-76,82}, 
rotation=360, 
lineColor={0,0,255}, 
extent={{-2,2},{2,-2}}), Ellipse(origin={68,10}, 
rotation=360, 
lineColor={0,0,255}, 
extent={{-2,2},{2,-2}}), Line(origin={26,42}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={18,42}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={12,42}, 
rotation=360, 
points={{-6,0},{6,0}}, 
color={255,0,255}), Line(origin={36,64}, 
rotation=360, 
points={{-10,-18},{10,2},{10,18}}, 
color={0,0,255}), Line(origin={36,20}, 
rotation=360, 
points={{-10,18},{10,-2},{10,-18}}, 
color={0,0,255}), Line(origin={36,28}, 
rotation=360, 
points={{6,-6},{0,6},{-6,0},{6,-6}}, 
color={0,0,255}), Line(origin={51,42}, 
rotation=360, 
points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
color={0,0,255}), Line(origin={56,50}, 
rotation=360, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Line(origin={56,42}, 
rotation=360, 
points={{0,8},{-10,-8},{10,-8},{0,8}}, 
color={0,0,255}), Line(origin={26,-38}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={18,-38}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}), Line(origin={12,-38}, 
rotation=360, 
points={{-6,0},{6,0}}, 
color={255,0,255}), Line(origin={36,-16}, 
rotation=360, 
points={{-10,-18},{10,2},{10,18}}, 
color={0,0,255}), Line(origin={36,-60}, 
rotation=360, 
points={{-10,18},{10,-2},{10,-18}}, 
color={0,0,255}), Line(origin={36,-52}, 
rotation=360, 
points={{6,-6},{0,6},{-6,0},{6,-6}}, 
color={0,0,255}), Line(origin={51,-38}, 
rotation=360, 
points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
color={0,0,255}), Line(origin={56,-30}, 
rotation=360, 
points={{-10,0},{10,0}}, 
color={0,0,255}), Line(origin={56,-38}, 
rotation=360, 
points={{0,8},{-10,-8},{10,-8},{0,8}}, 
color={0,0,255}), Line(origin={67,2}, 
rotation=360, 
points={{21,-1.77636e-15},{-21,1.77636e-15}}, 
color={0,0,255}), Ellipse(origin={88,2}, 
rotation=360, 
lineColor={0,0,255}, 
extent={{-2,2},{2,-2}}), Text(origin={-76,2.5}, 
lineColor={0,0,255}, 
extent={{-23,10},{23,-10}}, 
textString="DC", 
textColor={0,0,255}), Text(origin={88,-22}, 
lineColor={0,0,255}, 
extent={{-23,10},{23,-10}}, 
textString="AC", 
textColor={0,0,255})}));
    equation

      q = Functions.Func_q_column(GAM_Configuration,m01c,m01s,fai);

      V0 = voltageSensor.v;
      I0[1] = signalCurrent.i;
      Vac = signalVoltage_GAM.v;
      Iac = currentSensor_GAM.i;

      Vac = vector((2*q)*V0);
      I0 = 2*0.5*transpose(q) * Iac;

      connect(currentSensor_GAM.plug_p, resistor_GAM.plug_n) 
      annotation(Line(origin={69,27}, 
    points={{-3,33},{-15,33}}, 
    color={0,0,255}));
      connect(resistor_GAM.plug_p, signalVoltage_GAM.plug_p) 
      annotation(Line(origin={40,20}, 
    points={{-6,40},{-22,40},{-22,-10}}, 
    color={0,0,255}));
      connect(currentSensor_GAM.plug_n, ac_p) 
      annotation(Line(origin={86,54}, 
    points={{0,6},{14,6}}, 
    color={0,0,255}));
      connect(signalVoltage_GAM.plug_n, ac_n) 
      annotation(Line(origin={58,-35}, 
    points={{-40,25},{-40,-25},{42,-25}}, 
    color={0,0,255}));
      connect(dc_p, signalCurrent.p) 
      annotation(Line(origin={-89,35}, 
      points={{-11,25},{11,25},{11,-25}}, 
      color={0,0,255}));
      connect(signalCurrent.n, dc_n) 
      annotation(Line(origin={-89,-35}, 
      points={{11,25},{11,-25},{-11,-25}}, 
      color={0,0,255}));
      connect(voltageSensor.p, dc_p) 
      annotation(Line(origin={-70,35}, 
    points={{34,-25},{34,25},{-30,25}}, 
    color={0,0,255}));
      connect(voltageSensor.n, dc_n) 
      annotation(Line(origin={-70,-35}, 
    points={{34,25},{34,-25},{-30,-25}}, 
    color={0,0,255}));
      connect(ground_GAM.p, ac_n) 
      annotation(Line(origin={50,-62}, 
      points={{-50,-2.4},{-50,2},{50,2}}));
      end Full_Bridge_GAM;
    model Three_two_level_Converter_GAM "GAM of Three Phase Two Level Inverter"
      extends Interfaces_GAM.DCAC_three_phase_pin_GAM;
      parameter Real fai "载波信号的相位/rad";


      Real[GAM_Configuration,1] qa;
      Real[GAM_Configuration,1] qb;
      Real[GAM_Configuration,1] qc;

      Real V0;
      Real[GAM_Configuration] Van;
      Real[GAM_Configuration] Vbn;
      Real[GAM_Configuration] Vcn;
      Real[GAM_Configuration] Ian;
      Real[GAM_Configuration] Ibn;
      Real[GAM_Configuration] Icn;
      Real[1] I0;

      annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2})), Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Rectangle(origin={0,0}, 
lineColor={0,0,0}, 
fillColor={255,202,216}, 
fillPattern=FillPattern.Solid, 
extent={{-100,100},{100,-100}}), Line(origin={-30,-2}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}, 
thickness=0.5), Line(origin={-38,-2}, 
rotation=360, 
points={{0,20},{0,-20}}, 
color={0,0,255}, 
thickness=0.5), Line(origin={-47,-2}, 
rotation=360, 
points={{-9,0},{9,0}}, 
color={255,0,255}, 
thickness=0.5), Line(origin={-18,34}, 
rotation=360, 
points={{-12,-26},{12,-2},{12,26}}, 
color={0,0,255}, 
thickness=0.5), Line(origin={-17,-32}, 
rotation=360, 
points={{-13,26},{13,0},{13,-26}}, 
color={0,0,255}, 
thickness=0.5), Line(origin={-20,-16}, 
rotation=360, 
points={{8,-8},{-2,8},{-8,2},{8,-8}}, 
color={0,0,255}, 
thickness=0.5), Line(origin={3,0}, 
rotation=360, 
points={{-7,-40},{9,-40},{9,40},{-9,40}}, 
color={0,0,255}, 
thickness=0.5), Line(origin={13,12}, 
rotation=360, 
points={{-17,0},{17,0}}, 
color={0,0,255}, 
thickness=0.5), Line(origin={12,-3}, 
rotation=360, 
points={{0,15},{-16,-15},{16,-15},{0,15}}, 
color={0,0,255}, 
thickness=0.5), Text(origin={-2.82759,81.5}, 
lineColor={0,0,0}, 
extent={{-88,15.5},{88,-15.5}}, 
textString="3-Phase 2-Level", 
fontName="Courier New", 
textStyle={TextStyle.None}, 
textColor={0,0,0}), Text(origin={-41.5,49}, 
lineColor={0,0,255}, 
extent={{-28.25,19},{28.25,-19}}, 
textString="DC", 
textColor={0,0,255}), Text(origin={47.25,-38}, 
lineColor={0,0,255}, 
extent={{-28.25,19},{28.25,-19}}, 
textString="AC", 
textColor={0,0,255}), Text(origin={73.5862,58}, 
lineColor={0,0,0}, 
extent={{-11.5862,10.5},{11.5862,-10.5}}, 
textString="A", 
fontName="Times New Roman", 
textStyle={TextStyle.Bold}, 
textColor={0,0,0}), Text(origin={73.5862,0.5}, 
lineColor={0,0,0}, 
extent={{-11.5862,10.5},{11.5862,-10.5}}, 
textString="B", 
fontName="Times New Roman", 
textStyle={TextStyle.Bold}, 
textColor={0,0,0}), Text(origin={73.5862,-58.5}, 
lineColor={0,0,0}, 
extent={{-11.5862,10.5},{11.5862,-10.5}}, 
textString="C", 
fontName="Times New Roman", 
textStyle={TextStyle.Bold}, 
textColor={0,0,0}), Text(origin={-78.8276,-58.5}, 
lineColor={0,0,0}, 
extent={{-12,19},{12,-19}}, 
textString="-", 
fontName="Times New Roman", 
textStyle={TextStyle.Bold}, 
textColor={0,0,0}), Text(origin={-78.8276,60}, 
lineColor={0,0,0}, 
extent={{-12,14},{12,-14}}, 
textString="+", 
fontName="Times New Roman", 
textStyle={TextStyle.Bold}, 
textColor={0,0,0}), Text(origin={-6.21725e-15,-83}, 
lineColor={0,0,0}, 
extent={{-69.2069,19},{69.2068975,-19}}, 
textString="Order = %GAM_Configuration", 
textStyle={TextStyle.None}, 
textColor={0,0,0}, 
horizontalAlignment=LinePattern.None)}));
      Sensors_GAM.CurrentSensor_GAM currentSensor_GAM(GAM_Configuration = GAM_Configuration) 
        annotation(Placement(transformation(origin = {66, 60}, 
        extent = {{-10, -10}, {10, 10}})));
      Sensors_GAM.CurrentSensor_GAM currentSensor_GAM1(GAM_Configuration = GAM_Configuration) 
        annotation(Placement(transformation(origin = {74, 0}, 
        extent = {{-10, -10}, {10, 10}})));
      Sensors_GAM.CurrentSensor_GAM currentSensor_GAM2(GAM_Configuration = GAM_Configuration) 
        annotation(Placement(transformation(origin = {66, -60}, 
        extent = {{-10, -10}, {10, 10}})));
      Basic_GAM.Resistor_GAM resistor_GAM(GAM_Configuration = GAM_Configuration, R = 0.0001) 
        annotation(Placement(transformation(origin = {32, 60}, 
        extent = {{-10, -10}, {10, 10}})));
      Basic_GAM.Resistor_GAM resistor_GAM1(GAM_Configuration = GAM_Configuration, R = 0.0001) 
        annotation(Placement(transformation(origin = {44, -4.197e-22}, 
        extent = {{-10, -10}, {10, 10}})));
      Basic_GAM.Resistor_GAM resistor_GAM2(GAM_Configuration = GAM_Configuration, R = 0.0001) 
        annotation(Placement(transformation(origin = {32, -60}, 
        extent = {{-10, -10}, {10, 10}})));
      Sources_GAM.SignalVoltage_GAM signalVoltage_GAM(GAM_Configuration=GAM_Configuration) 
        annotation(Placement(transformation(origin = {-16, 28}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 270)));
      Sources_GAM.SignalVoltage_GAM signalVoltage_GAM1(GAM_Configuration=GAM_Configuration) 
        annotation(Placement(transformation(origin = {14, 1.11022e-16}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 180)));
      Sources_GAM.SignalVoltage_GAM signalVoltage_GAM2(GAM_Configuration=GAM_Configuration) 
        annotation(Placement(transformation(origin = {-16, -32}, 
        extent = {{10, -10}, {-10, 10}}, 
        rotation = -90)));
      Modelica.Blocks.Interfaces.RealInput ma01c "a相调制波信号的余弦项系数" 
        annotation(Placement(transformation(origin = {-70, 110}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 270)));
      Modelica.Blocks.Interfaces.RealInput ma01s "a相调制波信号的正弦项系数" 
        annotation(Placement(transformation(origin = {-42, 110}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 270)));
      Modelica.Blocks.Interfaces.RealInput mb01c "b相调制波信号的余弦项系数" 
        annotation(Placement(transformation(origin = {-14, 110}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 270)));
      Modelica.Blocks.Interfaces.RealInput mb01s "b相调制波信号的正弦项系数" 
        annotation(Placement(transformation(origin = {14, 110}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 270)));
      Modelica.Blocks.Interfaces.RealInput mc01c "c相调制波信号的余弦项系数" 
        annotation(Placement(transformation(origin = {42, 110}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 270)));
      Modelica.Blocks.Interfaces.RealInput mc01s "c相调制波信号的正弦项系数" 
        annotation(Placement(transformation(origin = {70, 110}, 
        extent = {{-10, -10}, {10, 10}}, 
        rotation = 270)));
      Basic_GAM.Ground_GAM ground_GAM(GAM_Configuration=GAM_Configuration) 
        annotation (Placement(transformation(origin={-30,-80}, 
    extent={{-10,-10},{10,10}})));
      Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent 
        annotation (Placement(transformation(origin={-80,0}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor 
        annotation (Placement(transformation(origin={-46,-4.197e-22}, 
    extent={{10,-10},{-10,10}}, 
    rotation=-270)));
    equation

      qa = Functions.Func_q_column(GAM_Configuration, ma01c, ma01s, fai);
      qb = Functions.Func_q_column(GAM_Configuration, mb01c, mb01s, fai);
      qc = Functions.Func_q_column(GAM_Configuration, mc01c, mc01s, fai);

      V0 = voltageSensor.v;
      I0[1] = signalCurrent.i;
      Van = signalVoltage_GAM.v;
      Ian = currentSensor_GAM.i;
      Vbn = signalVoltage_GAM1.v;
      Ibn = currentSensor_GAM1.i;
      Vcn = signalVoltage_GAM2.v;
      Icn = currentSensor_GAM2.i;

      Van = vector((2 * qa) * V0 / 2);
      Vbn = vector((2 * qb) * V0 / 2);
      Vcn = vector((2 * qc) * V0 / 2);
      I0 = 0.5 * (2*0.5*(transpose(qa)) * Ian + 2*0.5*(transpose(qb)) * Ibn + 2*0.5*(transpose(qc)) * Icn);

      connect(ac_pb, currentSensor_GAM1.plug_n) 
        annotation(Line(origin = {88, 0}, 
        points = {{12, 0}, {-4, 0}}, 
        color = {0, 0, 255}));
      connect(signalVoltage_GAM1.plug_p, resistor_GAM1.plug_p) 
        annotation(Line(origin = {15, 0}, 
        points = {{9, 1.11022e-16}, {19, 1.11022e-16}, {19, -4.197e-22}}, 
        color = {0, 0, 255}));
      connect(resistor_GAM1.plug_n, currentSensor_GAM1.plug_p) 
        annotation(Line(origin = {49, 0}, 
        points = {{5, -4.197e-22}, {15, -4.197e-22}, {15, 0}}, 
        color = {0, 0, 255}));
      connect(signalVoltage_GAM.plug_n, signalVoltage_GAM1.plug_n) 
        annotation(Line(origin = {-14, 9}, 
        points = {{-2, 9}, {-2, -9}, {18, -9}}, 
        color = {0, 0, 255}));
      connect(signalVoltage_GAM.plug_p, resistor_GAM.plug_p) 
        annotation(Line(origin = {3, 43}, 
        points = {{-19, -5}, {-19, 17}, {19, 17}}, 
        color = {0, 0, 255}));
      connect(resistor_GAM.plug_n, currentSensor_GAM.plug_p) 
        annotation(Line(origin = {49, 48}, 
        points = {{-7, 12}, {7, 12}}, 
        color = {0, 0, 255}));
      connect(ac_pa, currentSensor_GAM.plug_n) 
        annotation(Line(origin = {88, 60}, 
        points = {{12, 0}, {-12, 0}}, 
        color = {0, 0, 255}));
      connect(signalVoltage_GAM2.plug_n, signalVoltage_GAM1.plug_n) 
        annotation(Line(origin = {-14, -11}, 
        points = {{-2, -11}, {-2, 11}, {18, 11}}, 
        color = {0, 0, 255}));
      connect(signalVoltage_GAM2.plug_p, resistor_GAM2.plug_p) 
        annotation(Line(origin = {3, -47}, 
        points = {{-19, 5}, {-19, -13}, {19, -13}}, 
        color = {0, 0, 255}));
      connect(resistor_GAM2.plug_n, currentSensor_GAM2.plug_p) 
        annotation(Line(origin = {49, -60}, 
        points = {{-7, -7.10543e-15}, {7, 0}}, 
        color = {0, 0, 255}));
      connect(currentSensor_GAM2.plug_n, ac_pc) 
        annotation(Line(origin = {88, -60}, 
        points = {{-12, 0}, {12, 0}}, 
        color = {0, 0, 255}));
      connect(ground_GAM.p, signalVoltage_GAM1.plug_n) 
      annotation(Line(origin={-15,-35}, 
    points={{-15,-35.4},{-15,35},{19,35}}));
      connect(dc_p, signalCurrent.p) 
      annotation(Line(origin={-106,35}, 
    points={{6,25},{26,25},{26,-25}}, 
    color={0,0,255}));
      connect(signalCurrent.n, dc_n) 
      annotation(Line(origin={-106,-35}, 
    points={{26,25},{26,-25},{6,-25}}, 
    color={0,0,255}));
      connect(voltageSensor.p, dc_p) 
      annotation(Line(origin={-87,35}, 
    points={{41,-25},{41,25},{-13,25}}, 
    color={0,0,255}));
      connect(voltageSensor.n, dc_n) 
      annotation(Line(origin={-87,-35}, 
    points={{41,25},{41,-25},{-13,-25}}, 
    color={0,0,255}));
    end Three_two_level_Converter_GAM;
    model Full_Bridge_Inverter_Ave "Average Model of Full-Bridge PWM Inverter"
      extends PowerGAM.Interfaces_GAM.Converter1;
      extends PowerGAM.Interfaces_GAM.DCtwoPin;
      extends PowerGAM.Interfaces_GAM.ACtwoPin;
      Modelica.Blocks.Interfaces.RealInput U_ref 
        annotation (Placement(transformation(origin={0,100}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)));
      Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent 
        annotation (Placement(transformation(origin={-78,2.220446049250313e-16}, 
      extent={{-10,-10},{10,10}}, 
      rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor 
        annotation (Placement(transformation(origin={-40,3.3306690738754696e-16}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor 
        annotation (Placement(transformation(origin={66,60}, 
    extent={{-10,-10},{10,10}})),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.001) 
        annotation (Placement(transformation(origin={40,-30}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage 
        annotation (Placement(transformation(origin={40,16}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Product product1 
        annotation (Placement(transformation(origin={-16,34}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Product product2 
        annotation (Placement(transformation(origin={16,34}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2})),Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Rectangle(origin={-4.25,-4}, 
    lineColor={184,255,253}, 
    fillColor={184,255,253}, 
    fillPattern=FillPattern.Solid, 
    extent={{-83.75,90.5},{83.75,-90.5}}), Line(origin={-53.5,36}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={-61.5,36}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={-67.5,36}, 
    rotation=360, 
    points={{-6,0},{6,0}}, 
    color={255,0,255}), Line(origin={-43.5,58}, 
    rotation=360, 
    points={{-10,-18},{10,2},{10,18}}, 
    color={0,0,255}), Line(origin={-43.5,14}, 
    rotation=360, 
    points={{-10,18},{10,-2},{10,-18}}, 
    color={0,0,255}), Line(origin={-43.5,22}, 
    rotation=360, 
    points={{6,-6},{0,6},{-6,0},{6,-6}}, 
    color={0,0,255}), Line(origin={-28.5,36}, 
    rotation=360, 
    points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
    color={0,0,255}), Line(origin={-23.5,44}, 
    rotation=360, 
    points={{-10,0},{10,0}}, 
    color={0,0,255}), Line(origin={-23.5,36}, 
    rotation=360, 
    points={{0,8},{-10,-8},{10,-8},{0,8}}, 
    color={0,0,255}), Line(origin={-53.5,-44}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={-61.5,-44}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={-67.5,-44}, 
    rotation=360, 
    points={{-6,0},{6,0}}, 
    color={255,0,255}), Line(origin={-43.5,-22}, 
    rotation=360, 
    points={{-10,-18},{10,2},{10,18}}, 
    color={0,0,255}), Line(origin={-43.5,-66}, 
    rotation=360, 
    points={{-10,18},{10,-2},{10,-18}}, 
    color={0,0,255}), Line(origin={-43.5,-58}, 
    rotation=360, 
    points={{6,-6},{0,6},{-6,0},{6,-6}}, 
    color={0,0,255}), Line(origin={-28.5,-44}, 
    rotation=360, 
    points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
    color={0,0,255}), Line(origin={-23.5,-36}, 
    rotation=360, 
    points={{-10,0},{10,0}}, 
    color={0,0,255}), Line(origin={-23.5,-44}, 
    rotation=360, 
    points={{0,8},{-10,-8},{10,-8},{0,8}}, 
    color={0,0,255}), Line(origin={14.5,4}, 
    rotation=360, 
    points={{48,0},{-48,0}}, 
    color={0,0,255}), Line(origin={-20.5,76}, 
    rotation=360, 
    points={{61,0},{-61,0}}, 
    color={0,0,255}), Line(origin={-21.5,-84}, 
    rotation=360, 
    points={{62,0},{-62,0}}, 
    color={0,0,255}), Ellipse(origin={-83.5,-84}, 
    rotation=360, 
    lineColor={0,0,255}, 
    extent={{-2,2},{2,-2}}), Ellipse(origin={-81.5,76}, 
    rotation=360, 
    lineColor={0,0,255}, 
    extent={{-2,2},{2,-2}}), Ellipse(origin={62.5,4}, 
    rotation=360, 
    lineColor={0,0,255}, 
    extent={{-2,2},{2,-2}}), Line(origin={20.5,36}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={12.5,36}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={6.5,36}, 
    rotation=360, 
    points={{-6,0},{6,0}}, 
    color={255,0,255}), Line(origin={30.5,58}, 
    rotation=360, 
    points={{-10,-18},{10,2},{10,18}}, 
    color={0,0,255}), Line(origin={30.5,14}, 
    rotation=360, 
    points={{-10,18},{10,-2},{10,-18}}, 
    color={0,0,255}), Line(origin={30.5,22}, 
    rotation=360, 
    points={{6,-6},{0,6},{-6,0},{6,-6}}, 
    color={0,0,255}), Line(origin={45.5,36}, 
    rotation=360, 
    points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
    color={0,0,255}), Line(origin={50.5,44}, 
    rotation=360, 
    points={{-10,0},{10,0}}, 
    color={0,0,255}), Line(origin={50.5,36}, 
    rotation=360, 
    points={{0,8},{-10,-8},{10,-8},{0,8}}, 
    color={0,0,255}), Line(origin={20.5,-44}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={12.5,-44}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}), Line(origin={6.5,-44}, 
    rotation=360, 
    points={{-6,0},{6,0}}, 
    color={255,0,255}), Line(origin={30.5,-22}, 
    rotation=360, 
    points={{-10,-18},{10,2},{10,18}}, 
    color={0,0,255}), Line(origin={30.5,-66}, 
    rotation=360, 
    points={{-10,18},{10,-2},{10,-18}}, 
    color={0,0,255}), Line(origin={30.5,-58}, 
    rotation=360, 
    points={{6,-6},{0,6},{-6,0},{6,-6}}, 
    color={0,0,255}), Line(origin={45.5,-44}, 
    rotation=360, 
    points={{-5,-24},{5,-24},{5,24},{-5,24}}, 
    color={0,0,255}), Line(origin={50.5,-36}, 
    rotation=360, 
    points={{-10,0},{10,0}}, 
    color={0,0,255}), Line(origin={50.5,-44}, 
    rotation=360, 
    points={{0,8},{-10,-8},{10,-8},{0,8}}, 
    color={0,0,255}), Line(origin={61.5,-4}, 
    rotation=360, 
    points={{21,-1.77636e-15},{-21,1.77636e-15}}, 
    color={0,0,255}), Ellipse(origin={82.5,-4}, 
    rotation=360, 
    lineColor={0,0,255}, 
    extent={{-2,2},{2,-2}}), Text(origin={-81.5,-3.5}, 
    lineColor={0,0,255}, 
    extent={{-23,10},{23,-10}}, 
    textString="DC", 
    textColor={0,0,255}), Text(origin={82.5,-28}, 
    lineColor={0,0,255}, 
    extent={{-23,10},{23,-10}}, 
    textString="AC", 
    textColor={0,0,255}), Text(origin={29,90}, 
    lineColor={0,0,255}, 
    extent={{-23,10},{23,-10}}, 
    textString="q/Uref", 
    textColor={0,0,255})}));
    equation
      connect(dc_p, signalCurrent.p) 
      annotation(Line(origin={-89,35}, 
      points={{-11,25},{11,25},{11,-25}}, 
      color={0,0,255}));
      connect(signalCurrent.n, dc_n) 
      annotation(Line(origin={-89,-35}, 
      points={{11,25},{11,-25},{-11,-25}}, 
      color={0,0,255}));
      connect(ac_p, currentSensor.n) 
      annotation(Line(origin={85,60}, 
    points={{15,0},{-8.999999999999986,0}}, 
    color={0,0,255}));
      connect(signalVoltage.p, currentSensor.p) 
      annotation(Line(origin={45,46}, 
    points={{-5,-20},{-5,14},{11.000000000000007,14}}, 
    color={0,0,255}));
      connect(signalVoltage.n, resistor.p) 
      annotation(Line(origin={40,-2.9999999999999982}, 
    points={{0,8.999999999999998},{0,-17}}, 
    color={0,0,255}));
      connect(resistor.n, ac_n) 
      annotation(Line(origin={70,-43}, 
    points={{-30,3},{-30,-17.000000000000007},{30,-17.000000000000007}}, 
    color={0,0,255}));
      connect(voltageSensor.p, signalCurrent.p) 
      annotation(Line(origin={-60,35}, 
    points={{20,-25},{20,25},{-18,25},{-18,-25}}, 
    color={0,0,255}));
      connect(voltageSensor.n, dc_n) 
      annotation(Line(origin={-71,-35}, 
    points={{31,25},{31,-25},{-29,-25}}, 
    color={0,0,255}));
      connect(U_ref, product1.u1) 
      annotation(Line(origin={-5,80}, 
    points={{5,20},{5,0},{-5,0},{-5,-34}}, 
    color={0,0,127}));
      connect(product2.u2, U_ref) 
      annotation(Line(origin={5,73}, 
      points={{5,-27},{5,7},{-5,7},{-5,27}}, 
      color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
      connect(product1.y, signalVoltage.v) 
      annotation(Line(origin={24,9}, 
      points={{-40,14},{-40,-15},{40,-15},{40,7},{28,7}}, 
      color={0,0,127}));
      connect(currentSensor.i, product2.u1) 
      annotation(Line(origin={44,59}, 
      points={{22,-10},{22,-19},{2,-19},{2,19},{-22,19},{-22,-13}}, 
      color={0,0,127}));
      connect(ac_n, dc_n) 
      annotation(Line(origin={0,-60}, 
      points={{100,-7.105427357601002e-15},{-100,-7.105427357601002e-15},{-100,0}}, 
      color={0,0,255}));
      connect(voltageSensor.v, product1.u2) 
      annotation(Line(origin={-36,23}, 
    points={{-15,-23},{-20,-23},{-20,57},{14,57},{14,23}}, 
    color={0,0,127}));
      connect(product2.y, signalCurrent.i) 
      annotation(Line(origin={-25,12}, 
    points={{41,11},{41,-54},{-37,-54},{-37,-12},{-41,-12}}, 
    color={0,0,127}));
    end Full_Bridge_Inverter_Ave;
    model Three_Phase_2_Level_Inverter_Ave "Average Model of Three Phase Two Level Inverter"
      extends PowerGAM.Interfaces_GAM.Converter1;
      extends PowerGAM.Interfaces_GAM.DCtwoPin;
      extends PowerGAM.Interfaces_GAM.ACpin3;
      Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent 
        annotation (Placement(transformation(origin={-88,-4.440892098500626e-16}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor 
        annotation (Placement(transformation(origin={-53.9,0}, 
    extent={{-10,-10},{10,10}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Interfaces.RealInput Ua_ref 
        annotation (Placement(transformation(origin={-61.5,100}, 
    extent={{-10.5,-10.5},{10.5,10.5}}, 
    rotation=270)));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.001) 
        annotation (Placement(transformation(origin={54,34}, 
    extent={{-10,-10},{10,10}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor 
        annotation (Placement(transformation(origin={80,34}, 
    extent={{-10,-10},{10,10}})),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage 
        annotation (Placement(transformation(origin={25.5,20}, 
    extent={{10,-10},{-10,10}}, 
    rotation=-270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor1 
        annotation (Placement(transformation(origin={80,0}, 
    extent={{-10,-10},{10,10}})),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Basic.Resistor resistor1(R=0.001) 
        annotation (Placement(transformation(origin={54,0}, 
    extent={{-10,-10},{10,10}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor2 
        annotation (Placement(transformation(origin={80,-48}, 
    extent={{-10,-10},{10,10}})),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Basic.Resistor resistor2(R=0.001) 
        annotation (Placement(transformation(origin={54,-48}, 
    extent={{-10,-10},{10,10}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage1 
        annotation (Placement(transformation(origin={25.5,-34}, 
    extent={{-10,-10},{10,10}}, 
    rotation=90)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage2 
        annotation (Placement(transformation(origin={27.999999999999996,-5.551115123125783e-17}, 
    extent={{-10,-10},{10,10}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Interfaces.RealInput Ub_ref 
        annotation (Placement(transformation(origin={0,100}, 
    extent={{-10.5,-10.5},{10.5,10.5}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Interfaces.RealInput Uc_ref 
        annotation (Placement(transformation(origin={59.5,100}, 
    extent={{-10.5,-10.5},{10.5,10.5}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Product product1 
        annotation (Placement(transformation(origin={-57.9,74}, 
    extent={{-6,-6},{6,6}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Product product2 
        annotation (Placement(transformation(origin={-3.599999999999998,72.75}, 
    extent={{-6,-6},{6,6}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Product product3 
        annotation (Placement(transformation(origin={55.89999999999999,72.75}, 
    extent={{-6,-6},{6,6}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Sum sum1(nin=3) 
        annotation (Placement(transformation(origin={-22.499999999999986,-74.00000000000001}, 
    extent={{-6.000000000000002,-6},{6.000000000000002,6}}, 
    rotation=180)));
      Modelica.Blocks.Math.Product product7 
        annotation (Placement(transformation(origin={12,-56}, 
    extent={{-6,-6},{6,6}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Product product8 
        annotation (Placement(transformation(origin={12.399999999999999,-74}, 
    extent={{-6,-6},{6,6}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Product product9 
        annotation (Placement(transformation(origin={12.399999999999991,-92}, 
    extent={{-6,-6},{6,6}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Gain gain(k=0.5) 
        annotation (Placement(transformation(origin={-46,-74.00000000000003}, 
    extent={{-6,-6},{5.9999999999999964,5.9999999999999964}}, 
    rotation=180)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Gain gain1(k=0.5) 
        annotation (Placement(transformation(origin={-24,20}, 
    extent={{-6,-6},{5.9999999999999964,5.9999999999999964}}, 
    rotation=360)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Gain gain2(k=0.5) 
        annotation (Placement(transformation(origin={0,-1.3322676295501878e-15}, 
    extent={{-6,-6},{5.9999999999999964,5.9999999999999964}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Blocks.Math.Gain gain3(k=0.5) 
        annotation (Placement(transformation(origin={-9.999999999999996,-22.000000000000004}, 
    extent={{-6,-6},{5.9999999999999964,5.9999999999999964}}, 
    rotation=270)),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Basic.Ground ground 
        annotation (Placement(transformation(origin={-10,-130}, 
    extent={{-10,-10},{10,10}})));
      annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2})),Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Rectangle(origin={0,-9}, 
    lineColor={184,255,253}, 
    fillColor={184,255,253}, 
    fillPattern=FillPattern.Solid, 
    extent={{-88,79},{88,-79}}), Line(origin={-26,-1.999999999999993}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}, 
    thickness=0.5), Line(origin={-34,-1.999999999999993}, 
    rotation=360, 
    points={{0,20},{0,-20}}, 
    color={0,0,255}, 
    thickness=0.5), Line(origin={-43,-1.999999999999993}, 
    rotation=360, 
    points={{-9,0},{9,0}}, 
    color={255,0,255}, 
    thickness=0.5), Line(origin={-14,34.00000000000001}, 
    rotation=360, 
    points={{-12,-26},{12,-2},{12,26}}, 
    color={0,0,255}, 
    thickness=0.5), Line(origin={-13,-31.999999999999986}, 
    rotation=360, 
    points={{-13,26},{13,0},{13,-26}}, 
    color={0,0,255}, 
    thickness=0.5), Line(origin={-16,-15.999999999999993}, 
    rotation=360, 
    points={{8,-8},{-2,8},{-8,2},{8,-8}}, 
    color={0,0,255}, 
    thickness=0.5), Line(origin={7,7.105427357601002e-15}, 
    rotation=360, 
    points={{-7,-40},{9,-40},{9,40},{-9,40}}, 
    color={0,0,255}, 
    thickness=0.5), Line(origin={17,12.000000000000007}, 
    rotation=360, 
    points={{-17,0},{17,0}}, 
    color={0,0,255}, 
    thickness=0.5), Line(origin={16,-2.999999999999993}, 
    rotation=360, 
    points={{0,15},{-16,-15},{16,-15},{0,15}}, 
    color={0,0,255}, 
    thickness=0.5), Text(origin={4,-82.5}, 
    lineColor={0,0,0}, 
    extent={{-88,15.500000000000014},{88,-15.5}}, 
    textString="3-Phase 2-Level", 
    fontName="Courier New", 
    textStyle={TextStyle.None}, 
    textColor={0,0,0}), Text(origin={-37.5,49}, 
    lineColor={0,0,255}, 
    extent={{-28.25,19},{28.25,-19}}, 
    textString="DC", 
    textColor={0,0,255}), Text(origin={51.25,-38}, 
    lineColor={0,0,255}, 
    extent={{-28.25,19.000000000000004},{28.25,-19}}, 
    textString="AC", 
    textColor={0,0,255}), Text(origin={-61.49999999999999,84}, 
    lineColor={0,0,0}, 
    extent={{-20,14},{20,-14.000000000000007}}, 
    textString="Ua_ref", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,0}), Text(origin={77.58620689655172,58}, 
    lineColor={0,0,0}, 
    extent={{-11.586206896551715,10.5},{11.58620689655173,-10.5}}, 
    textString="A", 
    fontName="Times New Roman", 
    textStyle={TextStyle.Bold}, 
    textColor={0,0,0}), Text(origin={77.58620689655172,0.5}, 
    lineColor={0,0,0}, 
    extent={{-11.586206896551715,10.5},{11.58620689655173,-10.5}}, 
    textString="B", 
    fontName="Times New Roman", 
    textStyle={TextStyle.Bold}, 
    textColor={0,0,0}), Text(origin={77.58620689655172,-58.50000000000001}, 
    lineColor={0,0,0}, 
    extent={{-11.586206896551715,10.5},{11.58620689655173,-10.5}}, 
    textString="C", 
    fontName="Times New Roman", 
    textStyle={TextStyle.Bold}, 
    textColor={0,0,0}), Text(origin={-79.99999999999999,-58.49999999999999}, 
    lineColor={0,0,0}, 
    extent={{-12,19.000000000000007},{12.000000000000007,-19}}, 
    textString="-", 
    fontName="Times New Roman", 
    textStyle={TextStyle.Bold}, 
    textColor={0,0,0}), Text(origin={-79.99999999999999,60}, 
    lineColor={0,0,0}, 
    extent={{-12,14},{12,-14}}, 
    textString="+", 
    fontName="Times New Roman", 
    textStyle={TextStyle.Bold}, 
    textColor={0,0,0}), Text(origin={-8.881784197001252e-16,84}, 
    lineColor={0,0,0}, 
    extent={{-20,14},{20,-14.000000000000007}}, 
    textString="Ub_ref", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,0}), Text(origin={59.5,84}, 
    lineColor={0,0,0}, 
    extent={{-20,14},{20,-14.000000000000007}}, 
    textString="Uc_ref", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,0})}));
    equation
      connect(dc_p, signalCurrent.p) 
      annotation(Line(origin={-82,51}, 
    points={{-18,9},{-6,9},{-6,-41}}, 
    color={0,0,255}));
      connect(voltageSensor.p, dc_p) 
      annotation(Line(origin={-60,35}, 
    points={{6.100000000000001,-25},{6.100000000000001,25},{-40,25}}, 
    color={0,0,255}));
      connect(voltageSensor.n, dc_n) 
      annotation(Line(origin={-60,-35}, 
    points={{6.100000000000001,25},{6.100000000000001,-25},{-40,-25}}, 
    color={0,0,255}));
      connect(ac_a, currentSensor.n) 
      annotation(Line(origin={85,58}, 
    points={{15,0},{9,0},{9,-24},{5,-24}}, 
    color={0,0,255}));
      connect(currentSensor.p, resistor.p) 
      annotation(Line(origin={52,58}, 
    points={{18,-24},{12,-24}}, 
    color={0,0,255}));
      connect(resistor.n, signalVoltage.p) 
      annotation(Line(origin={22,50}, 
    points={{22,-16},{3.5,-16},{3.5,-20}}, 
    color={0,0,255}));
      connect(currentSensor1.p, resistor1.p) 
      annotation(Line(origin={55,0}, 
    points={{15,0},{9,0}}, 
    color={0,0,255}));
      connect(ac_b, currentSensor1.n) 
      annotation(Line(origin={90,0}, 
    points={{10,0},{0,0}}, 
    color={0,0,255}));
      connect(ac_c, currentSensor2.n) 
      annotation(Line(origin={90,-58}, 
    points={{10,0},{0,0},{0,10}}, 
    color={0,0,255}));
      connect(currentSensor2.p, resistor2.p) 
      annotation(Line(origin={55,-58}, 
    points={{15,10},{9,10}}, 
    color={0,0,255}));
      connect(resistor2.n, signalVoltage1.p) 
      annotation(Line(origin={22,-53}, 
    points={{22,5},{3.5,5},{3.5,9}}, 
    color={0,0,255}));
      connect(resistor1.n, signalVoltage2.p) 
      annotation(Line(origin={41,0}, 
      points={{3,0},{-3,-5.551115123125783e-17}}, 
      color={0,0,255}));
      connect(signalVoltage.n, signalVoltage2.n) 
      annotation(Line(origin={16,14}, 
    points={{9.5,-4},{9.5,-8},{-2,-8},{-2,-14},{1.9999999999999964,-14}}, 
    color={0,0,255}));
      connect(signalVoltage1.n, signalVoltage2.n) 
      annotation(Line(origin={16,-14}, 
    points={{9.5,-10},{9.5,-6},{-2,-6},{-2,14},{1.9999999999999964,14}}, 
    color={0,0,255}));
      connect(signalCurrent.n, dc_n) 
      annotation(Line(origin={-94,-35}, 
      points={{6,25},{6,-25},{-6,-25}}, 
      color={0,0,255}));
      connect(voltageSensor.v, product1.u1) 
      annotation(Line(origin={-56,43}, 
    points={{-8.900000000000006,-43},{-12,-43},{-12,21},{14,21},{14,43},{1.7000000000000028,43},{1.7000000000000028,38.2}}, 
    color={0,0,127}));
      connect(product2.u2, voltageSensor.v) 
      annotation(Line(origin={-38,43}, 
    points={{30.800000000000004,36.95},{30.800000000000004,43},{-4,43},{-4,21},{-30,21},{-30,-43},{-26.900000000000006,-43}}, 
    color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
      connect(product3.u2, voltageSensor.v) 
      annotation(Line(origin={-9,43}, 
    points={{61.29999999999999,36.95},{61.29999999999999,43},{29,43},{29,13},{-33,13},{-33,21.03931650685884},{-59,21.03931650685884},{-59,-43},{-55.900000000000006,-43}}, 
    color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
      connect(Ua_ref, product1.u2) 
      annotation(Line(origin={-61,91}, 
      points={{-0.5,9},{-0.5,-9.799999999999997}}, 
      color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
      connect(Ub_ref, product2.u1) 
      annotation(Line(origin={0,90}, 
      points={{0,10},{0,-10.049999999999997},{1.7763568394002505e-15,-10.049999999999997}}, 
      color={0,0,127}));
      connect(Uc_ref, product3.u1) 
      annotation(Line(origin={60,90}, 
      points={{-0.5,10},{-0.5,-10.049999999999997},{-0.5000000000000071,-10.049999999999997}}, 
      color={0,0,127}));
      connect(product7.y, sum1.u[1]) 
      annotation(Line(origin={24.500000000000014,-65}, 
    points={{-19.100000000000016,9},{-25,9},{-25,-9.000000000000014},{-39.8,-9.000000000000014}}, 
    color={0,0,127}));
      connect(product8.y, sum1.u[2]) 
      annotation(Line(origin={24.500000000000014,-74}, 
    points={{-18.700000000000017,0},{-39.8,0},{-39.8,-1.4210854715202004e-14}}, 
    color={0,0,127}));
      connect(product9.y, sum1.u[3]) 
      annotation(Line(origin={24.500000000000014,-83}, 
    points={{-18.700000000000024,-9},{-25,-9},{-25,8.999999999999986},{-39.8,8.999999999999986}}, 
    color={0,0,127}));
      connect(sum1.y, gain.u) 
      annotation(Line(origin={-34,-74}, 
      points={{4.900000000000013,-1.4210854715202004e-14},{-4.800000000000004,-2.842170943040401e-14}}, 
      color={0,0,127}));
      connect(gain.y, signalCurrent.i) 
      annotation(Line(origin={-64,-37}, 
      points={{11.400000000000006,-37.00000000000003},{-12,-37.00000000000003},{-12,37}}, 
      color={0,0,127}));
      connect(currentSensor.i, product7.u2) 
      annotation(Line(origin={50,-15}, 
      points={{30,38},{30,33},{-14,33},{-14,-37.4},{-30.8,-37.4}}, 
      color={0,0,127}));
      connect(currentSensor1.i, product8.u2) 
      annotation(Line(origin={50,-41}, 
    points={{30,30},{30,15},{-12,15},{-12,-29.400000000000006},{-30.400000000000002,-29.400000000000006}}, 
    color={0,0,127}));
      connect(currentSensor2.i, product9.u2) 
      annotation(Line(origin={50,-74}, 
      points={{30,15},{30,-14.400000000000006},{-30.40000000000001,-14.400000000000006}}, 
      color={0,0,127}));
      connect(product7.u1, Ua_ref) 
      annotation(Line(origin={-23,20}, 
    points={{42.2,-79.6},{47,-79.6},{47,-66},{-47,-66},{-47,66},{-38.5,66},{-38.5,80}}, 
    color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
      connect(product8.u1, Ub_ref) 
      annotation(Line(origin={55,16}, 
      points={{-35.400000000000006,-93.6},{55,-93.6},{55,94},{-41,94},{-41,70.72796605036504},{-55,70.72796605036504},{-55,84}}, 
      color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
      connect(product9.u1, Uc_ref) 
      annotation(Line(origin={71,2}, 
      points={{-51.400000000000006,-97.6},{51,-97.6},{51,83.72339501503262},{-11.5,83.72339501503262},{-11.5,98}}, 
      color={0,0,127}),__MWORKS(BlockSystem(NamedSignal)));
      connect(product1.y, gain1.u) 
      annotation(Line(origin={-45,44}, 
      points={{-12.899999999999999,23.400000000000006},{-12.899999999999999,-24},{13.8,-24}}, 
      color={0,0,127}));
      connect(gain1.y, signalVoltage.v) 
      annotation(Line(origin={-2,20}, 
      points={{-15.400000000000002,0},{15.5,0}}, 
      color={0,0,127}));
      connect(product2.y, gain2.u) 
      annotation(Line(origin={-2,37}, 
      points={{-1.5999999999999979,29.150000000000006},{-1.5999999999999979,-27},{1.9999999999999982,-27},{1.9999999999999982,-29.8}}, 
      color={0,0,127}));
      connect(gain2.y, signalVoltage2.v) 
      annotation(Line(origin={14,-12}, 
      points={{-14.000000000000002,5.400000000000004},{-14.000000000000002,-6},{13.999999999999996,-6},{13.999999999999996,0}}, 
      color={0,0,127}));
      connect(gain3.y, signalVoltage1.v) 
      annotation(Line(origin={2,-31}, 
      points={{-11.999999999999998,2.400000000000002},{-11.999999999999998,-3},{11.5,-3}}, 
      color={0,0,127}));
      connect(gain3.u, product3.y) 
      annotation(Line(origin={23,26}, 
      points={{-33,-40.8},{-33,24},{32.89999999999999,24},{32.89999999999999,40.150000000000006}}, 
      color={0,0,127}));
      connect(ground.p, signalVoltage2.n) 
      annotation(Line(origin={4,-60}, 
      points={{-14,-60},{-14,20},{6,20},{6,60},{14,60}}, 
      color={0,0,255}));
      end Three_Phase_2_Level_Inverter_Ave;
    end PowerConverters_GAM;
  package Sensors_GAM "Sensor Model Library"
    annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2}),graphics = {Rectangle(origin={0,0}, 
lineColor={200,200,200}, 
fillColor={248,248,248}, 
fillPattern=FillPattern.HorizontalCylinder, 
extent={{-100,-100},{100,100}}, 
radius=25), Rectangle(origin={0,0}, 
lineColor={128,128,128}, 
extent={{-100,-100},{100,100}}, 
radius=25), Ellipse(origin={0,-30}, 
fillColor={255,255,255}, 
extent={{-90,-90},{90,90}}, 
startAngle=20, 
endAngle=160), Ellipse(origin={0,-30}, 
fillColor={128,128,128}, 
pattern=LinePattern.None, 
fillPattern=FillPattern.Solid, 
extent={{-20,-20},{20,20}}), Line(origin={0,-30}, 
points={{0,60},{0,90}}), Ellipse(origin={0,-30}, 
fillColor={64,64,64}, 
pattern=LinePattern.None, 
fillPattern=FillPattern.Solid, 
extent={{-10,-10},{10,10}}), Polygon(origin={0,-30}, 
rotation=-35, 
fillColor={64,64,64}, 
pattern=LinePattern.None, 
fillPattern=FillPattern.Solid, 
points={{-7,0},{-3,85},{0,90},{3,85},{7,0}})}));
    model VoltageSensor_GAM "Measurement of Fourier series components of voltage"
      extends Modelica.Icons.RoundSensor;
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      PowerGAM.Interfaces_GAM.PositivePlug_GAM plug_p(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{-110,-10},{-90,10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM plug_n(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealOutput v[GAM_Configuration](each unit="V") 
        "Voltage between pin p and n (= p.v - n.v) as output signal" 
        annotation (Placement(transformation(
            origin={0,-110}, 
            extent={{10,-10},{-10,10}}, 
            rotation=90)));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor[GAM_Configuration] 
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation
      connect(voltageSensor.n, plug_n.pin) 
        annotation (Line(points={{10,0},{100,0}}, color={0,0,255}));
      connect(voltageSensor.p, plug_p.pin) 
        annotation (Line(points={{-10,0},{-100,0}}, color={0,0,255}));
      connect(voltageSensor.v, v) annotation (Line(
          points={{0,-11},{0,-110}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Line(origin={-80,0}, 
    points={{10,0},{-10,0}}), Line(origin={80,0}, 
    points={{-10,0},{10,0}}), Line(origin={0,-85}, 
    points={{0,-15},{0,15}}, 
    color={0,0,127}), Text(origin={0,98}, 
    extent={{150,-15},{-150,15}}, 
    textString="Order =%GAM_Configuration"), Text(origin={0,-40}, 
    lineColor={64,64,64}, 
    extent={{-30,30},{30,-30}}, 
    textString="V", 
    textColor={64,64,64}), Text(origin={0,-96}, 
    lineColor={0,0,255}, 
    extent={{-78,26},{78,-26}}, 
    textString="%name", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,255})}),Documentation(info="<html>
<p>
Contains m voltage sensors (Modelica.Electrical.Analog.Sensors.VoltageSensor),
thus measuring the m potential differences <em>v[m]</em> between the m pins of plug_p and plug_n.
</p>
</html>"                    ));
    end VoltageSensor_GAM;
    model CurrentSensor_GAM "Measurement of Fourier series components of current"
      extends Modelica.Icons.RoundSensor;
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      PowerGAM.Interfaces_GAM.PositivePlug_GAM plug_p(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{-110,-10},{-90,10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM plug_n(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealOutput i[GAM_Configuration](each unit="A") 
        "Current in the branch from p to n as output signal" annotation (
          Placement(transformation(
            origin={0,-110}, 
            extent={{10,-10},{-10,10}}, 
            rotation=90)));
      Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor[GAM_Configuration] 
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation
      connect(plug_p.pin, currentSensor.p) 
        annotation (Line(points={{-100,0},{-10,0}}, color={0,0,255}));
      connect(currentSensor.n, plug_n.pin) 
        annotation (Line(points={{10,0},{100,0}}, color={0,0,255}));
      connect(currentSensor.i, i) annotation (Line(
          points={{0,-11},{0,-110}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Line(origin={0,-85}, 
    points={{0,-15},{0,15}}, 
    color={0,0,127}), Text(origin={0,-40}, 
    lineColor={64,64,64}, 
    extent={{-30,30},{30,-30}}, 
    textString="A", 
    textColor={64,64,64}), Line(origin={0,0}, 
    points={{100,0},{-100,0}}, 
    color={0,0,255}), Text(origin={0,93}, 
    extent={{150,-15},{-150,15}}, 
    textString="Order =%GAM_Configuration"), Text(origin={0,-102}, 
    lineColor={0,0,255}, 
    extent={{-78,26},{78,-26}}, 
    textString="%name", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,255})}), 
                                      Documentation(info="<html>
<p>
Contains m current sensors (Modelica.Electrical.Analog.Sensors.CurrentSensor),
thus measuring the m currents <em>i[m]</em> flowing from the m pins of plug_p to the m pins of plug_n.
</p>
</html>"                    ));
    end CurrentSensor_GAM;
    model VoltageSensor "Measurement of voltage value of GAM model"
      extends Modelica.Icons.RoundSensor;
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      parameter Real w_sw "开关频率";
      parameter Real w "系统频率";
      PowerGAM.Interfaces_GAM.PositivePlug_GAM plug_p(final GAM_Configuration = GAM_Configuration) annotation(Placement(
        transformation(extent = {{-110, -10}, {-90, 10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM plug_n(final GAM_Configuration = GAM_Configuration) annotation(Placement(
        transformation(extent = {{90, -10}, {110, 10}})));
      Modelica.Blocks.Interfaces.RealOutput V_GAM[1](each unit = "V") 
        annotation(Placement(transformation(
        origin = {0, -110}, 
        extent = {{10, -10}, {-10, 10}}, 
        rotation = 90)));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor[GAM_Configuration] 
        annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
      Real c[1,GAM_Configuration];
      Real v[GAM_Configuration];
    equation
      c = Functions.Func_c(GAM_Configuration, w_sw, w, time);
      V_GAM = c * v;
      connect(voltageSensor.n, plug_n.pin) 
        annotation(Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 255}));
      connect(voltageSensor.p, plug_p.pin) 
        annotation(Line(points = {{-10, 0}, {-100, 0}}, color = {0, 0, 255}));
      voltageSensor.v = v;
      annotation(
        Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Line(origin={-80,0}, 
    points={{10,0},{-10,0}}), Line(origin={80,0}, 
    points={{-10,0},{10,0}}), Line(origin={0,-85}, 
    points={{0,-15},{0,15}}, 
    color={0,0,127}), Text(origin={0,90}, 
    extent={{152,-26},{-152,26}}, 
    textString="Order  = %GAM_Configuration"), Text(origin={0,-40}, 
    lineColor={64,64,64}, 
    extent={{-30,30},{30,-30}}, 
    textString="V", 
    textColor={64,64,64}), Text(origin={0,-96}, 
    lineColor={0,0,255}, 
    extent={{-78,26},{78,-26}}, 
    textString="%name", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,255})}), Documentation(info = "<html>
<p>
Contains m voltage sensors (Modelica.Electrical.Analog.Sensors.VoltageSensor),
thus measuring the m potential differences <em>v[m]</em> between the m pins of plug_p and plug_n.
</p>
</html>"                        ));
    end VoltageSensor;
    model CurrentSensor "Measurement of current value of GAM model"
      extends Modelica.Icons.RoundSensor;
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      parameter Real w_sw "开关频率";
      parameter Real w "系统频率";
      PowerGAM.Interfaces_GAM.PositivePlug_GAM plug_p(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{-110,-10},{-90,10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM plug_n(final GAM_Configuration=GAM_Configuration) annotation (Placement(
            transformation(extent={{90,-10},{110,10}})));
      Modelica.Blocks.Interfaces.RealOutput I_GAM[1](each unit="A") 
         annotation (
          Placement(transformation(
            origin={0,-110}, 
            extent={{10,-10},{-10,10}}, 
            rotation=90)));
      Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor[GAM_Configuration] 
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Real c[1,GAM_Configuration];
      Real i[GAM_Configuration];
    equation
      c = Functions.Func_c(GAM_Configuration, w_sw, w, time);
      I_GAM = c * i;
      connect(plug_p.pin, currentSensor.p) 
        annotation (Line(points={{-100,0},{-10,0}}, color={0,0,255}));
      connect(currentSensor.n, plug_n.pin) 
        annotation (Line(points={{10,0},{100,0}}, color={0,0,255}));
      currentSensor.i = i;
      annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Line(origin={0,-85}, 
    points={{0,-15},{0,15}}, 
    color={0,0,127}), Text(origin={0,-40}, 
    lineColor={64,64,64}, 
    extent={{-30,30},{30,-30}}, 
    textString="A", 
    textColor={64,64,64}), Line(origin={0,0}, 
    points={{100,0},{-100,0}}, 
    color={0,0,255}), Text(origin={0,93}, 
    extent={{150,-15},{-150,15}}, 
    textString="Order  = %GAM_Configuration"), Text(origin={0,-102}, 
    lineColor={0,0,255}, 
    extent={{-78,26},{78,-26}}, 
    textString="%name", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,255})}), 
                                      Documentation(info="<html>
<p>
Contains m current sensors (Modelica.Electrical.Analog.Sensors.CurrentSensor),
thus measuring the m currents <em>i[m]</em> flowing from the m pins of plug_p to the m pins of plug_n.
</p>
</html>"                    ));
    end CurrentSensor;

  end Sensors_GAM;

  package Interfaces_GAM "Interface Model Library"
    extends Modelica.Icons.InterfacesPackage;
    connector Plug_GAM "GAM of Plug"
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      Modelica.Electrical.Analog.Interfaces.Pin pin[GAM_Configuration] "Pins of the plug";
      annotation(Documentation(info = "<html>
<p>
Connectors PositivePlug and NegativePlug are nearly identical.
The only difference is that the icons are different in order
to identify more easily the plugs of a component.
Usually, connector PositivePlug is used for the positive and
connector NegativePlug for the negative plug of an electrical component.<br>
Connector Plug is a composite connector containing m Pins (Modelica.Electrical.Analog.Interfaces.Pin).
</p>
</html>"                                                ), 
        Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Text(origin={0,-139}, 
    lineColor={0,0,255}, 
    extent={{-100,40},{100,-40}}, 
    textString="%name", 
    textColor={0,0,255}), Ellipse(origin={0,0}, 
    lineColor={0,0,255}, 
    fillColor={0,0,255}, 
    fillPattern=FillPattern.Solid, 
    extent={{-40,40},{40,-40}})}));
    end Plug_GAM;
    connector PositivePlug_GAM "GAM of Positive Plug"
      extends PowerGAM.Interfaces_GAM.Plug_GAM;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Ellipse(
              extent={{-100,100},{100,-100}}, 
              lineColor={0,0,255}, 
              fillColor={0,0,255}, 
              fillPattern=FillPattern.Solid)}), 
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100}, 
                {100,100}}), graphics={Ellipse(
              extent={{-40,40},{40,-40}}, 
              lineColor={0,0,255}, 
              fillColor={0,0,255}, 
              fillPattern=FillPattern.Solid)}), 
        Documentation(info="<html>
<p>
Connectors PositivePlug and NegativePlug are nearly identical.
The only difference is that the icons are different in order
to identify more easily the plugs of a component.
Usually, connector PositivePlug is used for the positive and
connector NegativePlug for the negative plug of an electrical component.<br>
Connector Plug is a composite connector containing m Pins (Modelica.Electrical.Analog.Interfaces.Pin).
</p>
</html>"                ));
    end PositivePlug_GAM;
    connector NegativePlug_GAM "GAM of Negative Plug"
      extends PowerGAM.Interfaces_GAM.Plug_GAM;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                100,100}}), graphics={Ellipse(
              extent={{-100,100},{100,-100}}, 
              lineColor={0,0,255}, 
              fillColor={255,255,255}, 
              fillPattern=FillPattern.Solid)}), 
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100}, 
                {100,100}}), graphics={Ellipse(
              extent={{-40,40},{40,-40}}, 
              lineColor={0,0,255}, 
              fillColor={255,255,255}, 
              fillPattern=FillPattern.Solid)}), 
        Documentation(info="<html>
<p>
Connectors PositivePlug and NegativePlug are nearly identical.
The only difference is that the icons are different in order
to identify more easily the plugs of a component.
Usually, connector PositivePlug is used for the positive and
connector NegativePlug for the negative plug of an electrical component.<br>
Connector Plug is a composite connector containing m Pins (Modelica.Electrical.Analog.Interfaces.Pin).
</p>
</html>"                ));
    end NegativePlug_GAM;
    partial model TwoPlug_GAM "GAM of Two Plugs"
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      Modelica.Units.SI.Voltage v[GAM_Configuration] "Voltage drops of the two polyphase plugs";
      Modelica.Units.SI.Current i[GAM_Configuration] "Currents flowing into positive polyphase plugs";
      PowerGAM.Interfaces_GAM.PositivePlug_GAM plug_p(final GAM_Configuration=GAM_Configuration) "Positive polyphase electrical plug_GAM" annotation (Placement(transformation(origin={-100,0}, 
    extent={{-10,-10},{10,10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM plug_n(final GAM_Configuration=GAM_Configuration) "Negative polyphase electrical plug_GAM" annotation (Placement(transformation(origin={100,0}, 
    extent={{-10,-10},{10,10}})));
    equation
      v = plug_p.pin.v - plug_n.pin.v;
      i = plug_p.pin.i;
      annotation (Documentation(info="<html>
<p>
Superclass of elements which have <strong>two</strong> electrical plugs:
the positive plug connector <em>plug_p</em>, and the negative plug connector <em>plug_n</em>.
The currents flowing into plug_p are provided explicitly as currents i[m].
</p>
</html>"                                    ));
    end TwoPlug_GAM;
    partial model OnePort_GAM 
      "GAM of One Port"
      extends PowerGAM.Interfaces_GAM.TwoPlug_GAM;
    equation
      plug_p.pin.i + plug_n.pin.i = zeros(GAM_Configuration);
      annotation (Documentation(info="<html>
<p>
Superclass of elements which have <strong>two</strong> electrical plugs:
the positive plug connector <em>plug_p</em>, and the negative plug connector <em>plug_n</em>.
The currents flowing into plug_p are provided explicitly as currents i[m].
It is assumed that the currents flowing into plug_p are identical to the currents flowing out of plug_n.
</p>
</html>"            ));
    end OnePort_GAM;
    partial model DCACfourpin_GAM "GAM of One Phase Inverter Plugs"
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      PowerGAM.Interfaces_GAM.PositivePlug_GAM ac_p(GAM_Configuration = GAM_Configuration) 
        annotation (Placement(transformation(origin={100,60}, 
    extent={{-10,-10},{10,10}})));
      PowerGAM.Interfaces_GAM.NegativePlug_GAM ac_n(GAM_Configuration = GAM_Configuration) 
        annotation (Placement(transformation(origin={100,-60}, 
    extent={{-10,-10},{10,10}})));

      Modelica.Units.SI.Voltage vDC=dc_p.v - dc_n.v "DC voltage side";
      Modelica.Units.SI.Current iDC=dc_p.i "DC current side";
      Modelica.Units.SI.Voltage[GAM_Configuration] vAC=ac_p.pin.v - ac_n.pin.v "AC voltage side GAM";
      Modelica.Units.SI.Current[GAM_Configuration] iAC=ac_p.pin.i "AC current side GAM";
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p 
        annotation (Placement(transformation(origin={-100,60}, 
    extent={{-10,-10},{10,10}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n 
        annotation (Placement(transformation(origin={-100,-60}, 
    extent={{-10,-10},{10,10}})));
    end DCACfourpin_GAM;
    partial model DCAC_three_phase_pin_GAM "GAM of Three Phase Inverter Plugs"
      parameter Integer GAM_Configuration(min = 2, max = 16)=2 "GAM 模型的配置设置" 
        annotation (Evaluate = true, choices(choice = 2 "基波频率", choice = 6 "基波+边带频率", choice = 8 "基波+开关+边带频率", choice = 14 "基波+边带+2倍边带", choice = 16 "基波+开关+边带+2倍边带"));
      PowerGAM.Interfaces_GAM.PositivePlug_GAM ac_pa(GAM_Configuration = GAM_Configuration) 
        annotation (Placement(transformation(origin={100,60}, 
    extent={{-10,-10},{10,10}})));
      PositivePlug_GAM ac_pb(GAM_Configuration = GAM_Configuration) 
        annotation (Placement(transformation(origin={100,0}, 
    extent={{-10,-10},{10,10}})));
      PositivePlug_GAM ac_pc(GAM_Configuration = GAM_Configuration) 
        annotation (Placement(transformation(origin={100,-60}, 
    extent={{-10,-10},{10,10}})));
      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p 
        annotation (Placement(transformation(origin={-100,60}, 
    extent={{-10,-10},{10,10}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n 
        annotation (Placement(transformation(origin={-100,-60}, 
    extent={{-10,-10},{10,10}})));
      annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2})));
    equation
      end DCAC_three_phase_pin_GAM;
    partial model DCtwoPin "Positive and negative DC pins"

      Modelica.Electrical.Analog.Interfaces.PositivePin dc_p 
        "Positive DC input" 
        annotation (Placement(transformation(origin={-100,60}, 
    extent={{-10,10},{10,-10}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin dc_n 
        "Negative DC input" 
        annotation (Placement(transformation(origin={-100,-60}, 
    extent={{-10,-10},{10,10}})));
      Modelica.Units.SI.Voltage vDC=dc_p.v - dc_n.v "DC voltage";
      Modelica.Units.SI.Current iDC=dc_p.i "DC current";
      Modelica.Units.SI.Power powerDC=vDC*iDC "DC power";
    end DCtwoPin;
    partial model ACtwoPin "Positive and negative AC pin"

      Modelica.Electrical.Analog.Interfaces.PositivePin ac_p 
        "Positive AC input" 
        annotation (Placement(transformation(origin={100,60}, 
    extent={{-10,-10},{10,10}})));
      Modelica.Electrical.Analog.Interfaces.NegativePin ac_n 
        "Negative AC input" 
        annotation (Placement(transformation(origin={100,-60}, 
    extent={{-10,-10},{10,10}})));
      Modelica.Units.SI.Voltage vAC=ac_p.v - ac_n.v "AC voltages";
      Modelica.Units.SI.Current iAC=ac_p.i "AC currents";
      Modelica.Units.SI.Power powerAC=vAC*iAC "AC power";
    end ACtwoPin;
    partial model ACpin3 "Single AC pin 3 phase"

      Modelica.Electrical.Analog.Interfaces.PositivePin ac_a "AC output" 
        annotation (Placement(transformation(origin={100,0}, 
    extent={{-10,48},{10,68}})));
      Modelica.Units.SI.Voltage vAC=ac_a.v "AC potential";
      Modelica.Units.SI.Current iAC=ac_a.i "AC current";
      Modelica.Units.SI.Power powerAC=vAC*iAC "AC power";
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_b "AC output" 
        annotation (Placement(transformation(origin={100,-58}, 
    extent={{-10,48},{10,68}})),__MWORKS(BlockSystem(StateMachine)));
      Modelica.Electrical.Analog.Interfaces.PositivePin ac_c "AC output" 
        annotation (Placement(transformation(origin={100,-116}, 
    extent={{-10,48},{10,68}})),__MWORKS(BlockSystem(StateMachine)));
      annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2})));
    end ACpin3;
    model Converter1
      annotation(Icon(coordinateSystem(extent={{-100,-100},{100,100}}, 
    grid={2,2}),graphics = {Rectangle(origin={0,-2}, 
    fillColor={184,255,253}, 
    fillPattern=FillPattern.Solid, 
    extent={{-100,102},{100,-102}}), Line(origin={61,61}, 
    points={{-39,-39},{39,39}}, 
    color={0,0,127}), Text(origin={3.552713678800501e-15,-144.5}, 
    lineColor={0,0,255}, 
    extent={{-79.5,38.5},{79.5,-38.5}}, 
    textString="%name", 
    fontName="Times New Roman", 
    textStyle={TextStyle.None}, 
    textColor={0,0,255}), Line(origin={-65,-68}, 
    points={{-35,-36},{35,36}}, 
    color={0,0,127})}));

    end Converter1;
  end Interfaces_GAM;

  package Functions "Function Library"
    extends Modelica.Icons.FunctionsPackage;
    function Func_q_column "Calculation of GAM switching function"
      extends Modelica.Icons.Function;
      import Modelica.Constants.pi;
      input Integer Conf;
      input Real m01c;
      input Real m01s;
      input Real fai;
      output Real[Conf,1] q;
    algorithm
      if Conf == 2 then
        q[1,1] := Functions.Func_q01c(m01c);
        q[2,1] := Functions.Func_q01s(m01s);
      end if;

      if Conf == 6 then
        q[1,1] := Functions.Func_q01c(m01c);
        q[2,1] := Functions.Func_q01s(m01s);
        q[3,1] := Functions.Func_qnic(1,-2,m01c,m01s,fai);
        q[4,1] := Functions.Func_qnis(1,-2,m01c,m01s,fai);
        q[5,1] := Functions.Func_qnic(1,2,m01c,m01s,fai);
        q[6,1] := Functions.Func_qnis(1,2,m01c,m01s,fai);
      end if;

      if Conf == 8 then
        q[1,1] := Functions.Func_q01c(m01c);
        q[2,1] := Functions.Func_q01s(m01s);
        q[3,1] := Functions.Func_qnic(1,0,m01c,m01s,fai);
        q[4,1] := Functions.Func_qnis(1,0,m01c,m01s,fai);
        q[5,1] := Functions.Func_qnic(1,-2,m01c,m01s,fai);
        q[6,1] := Functions.Func_qnis(1,-2,m01c,m01s,fai);
        q[7,1] := Functions.Func_qnic(1,2,m01c,m01s,fai);
        q[8,1] := Functions.Func_qnis(1,2,m01c,m01s,fai);
      end if;

      if Conf == 14 then
        q[1,1] := Functions.Func_q01c(m01c);
        q[2,1] := Functions.Func_q01s(m01s);
        q[3,1] := Functions.Func_qnic(1,-2,m01c,m01s,fai);
        q[4,1] := Functions.Func_qnis(1,-2,m01c,m01s,fai);
        q[5,1] := Functions.Func_qnic(1,2,m01c,m01s,fai);
        q[6,1] := Functions.Func_qnis(1,2,m01c,m01s,fai);
        q[7,1] := Functions.Func_qnic(2,-1,m01c,m01s,fai);
        q[8,1] := Functions.Func_qnis(2,-1,m01c,m01s,fai);
        q[9,1] := Functions.Func_qnic(2,1,m01c,m01s,fai);
        q[10,1] := Functions.Func_qnis(2,1,m01c,m01s,fai);
        q[11,1] := Functions.Func_qnic(2,-3,m01c,m01s,fai);
        q[12,1] := Functions.Func_qnis(2,-3,m01c,m01s,fai);
        q[13,1] := Functions.Func_qnic(2,3,m01c,m01s,fai);
        q[14,1] := Functions.Func_qnis(2,3,m01c,m01s,fai);
      end if;

      if Conf == 16 then
        q[1,1] := Functions.Func_q01c(m01c);
        q[2,1] := Functions.Func_q01s(m01s);
        q[3,1] := Functions.Func_qnic(1,0,m01c,m01s,fai);
        q[4,1] := Functions.Func_qnis(1,0,m01c,m01s,fai);
        q[5,1] := Functions.Func_qnic(1,-2,m01c,m01s,fai);
        q[6,1] := Functions.Func_qnis(1,-2,m01c,m01s,fai);
        q[7,1] := Functions.Func_qnic(1,2,m01c,m01s,fai);
        q[8,1] := Functions.Func_qnis(1,2,m01c,m01s,fai);
        q[9,1] := Functions.Func_qnic(2,-1,m01c,m01s,fai);
        q[10,1] := Functions.Func_qnis(2,-1,m01c,m01s,fai);
        q[11,1] := Functions.Func_qnic(2,1,m01c,m01s,fai);
        q[12,1] := Functions.Func_qnis(2,1,m01c,m01s,fai);
        q[13,1] := Functions.Func_qnic(2,-3,m01c,m01s,fai);
        q[14,1] := Functions.Func_qnis(2,-3,m01c,m01s,fai);
        q[15,1] := Functions.Func_qnic(2,3,m01c,m01s,fai);
        q[16,1] := Functions.Func_qnis(2,3,m01c,m01s,fai);
      end if;

    end Func_q_column;
    function Func_c "Calculation of vector C"
      extends Modelica.Icons.Function;
      input Integer Conf;
      input Real w_sw;
      input Real w;
      input Real t;
      output Real[1, Conf] c;
    algorithm
      if Conf == 2 then
        c[1,1] := cos(w*t);
        c[1,2] := sin(w*t);
      end if;
      if Conf == 8 then
        c[1,1] := cos(w*t);
        c[1,2] := sin(w*t);
        c[1,3] := cos(w_sw*t);
        c[1,4] := sin(w_sw*t);
        c[1,5] := cos(w_sw*t - 2*w*t);
        c[1,6] := sin(w_sw*t - 2*w*t);
        c[1,7] := cos(w_sw*t + 2*w*t);
        c[1,8] := sin(w_sw*t + 2*w*t);
      end if;
      if Conf == 6 then
        c[1,1] := cos(w*t);
        c[1,2] := sin(w*t);
        c[1,3] := cos(w_sw*t - 2*w*t);
        c[1,4] := sin(w_sw*t - 2*w*t);
        c[1,5] := cos(w_sw*t + 2*w*t);
        c[1,6] := sin(w_sw*t + 2*w*t);
      end if;
      if Conf == 14 then
        c[1,1] := cos(w*t);
        c[1,2] := sin(w*t);
        c[1,3] := cos(w_sw*t - 2*w*t);
        c[1,4] := sin(w_sw*t - 2*w*t);
        c[1,5] := cos(w_sw*t + 2*w*t);
        c[1,6] := sin(w_sw*t + 2*w*t);
        c[1,7] := cos(2*w_sw*t - w*t);
        c[1,8] := sin(2*w_sw*t - w*t);
        c[1,9] := cos(2*w_sw*t + w*t);
        c[1,10] := sin(2*w_sw*t + w*t);
        c[1,11] := cos(2*w_sw*t - 3*w*t);
        c[1,12] := sin(2*w_sw*t - 3*w*t);
        c[1,13] := cos(2*w_sw*t + 3*w*t);
        c[1,14] := sin(2*w_sw*t + 3*w*t);
      end if;
      if Conf == 16 then
        c[1,1] := cos(w*t);
        c[1,2] := sin(w*t);
        c[1,3] := cos(w_sw*t);
        c[1,4] := sin(w_sw*t);
        c[1,5] := cos(w_sw*t - 2*w*t);
        c[1,6] := sin(w_sw*t - 2*w*t);
        c[1,7] := cos(w_sw*t + 2*w*t);
        c[1,8] := sin(w_sw*t + 2*w*t);
        c[1,9] := cos(2*w_sw*t - w*t);
        c[1,10] := sin(2*w_sw*t - w*t);
        c[1,11] := cos(2*w_sw*t + w*t);
        c[1,12] := sin(2*w_sw*t + w*t);
        c[1,13] := cos(2*w_sw*t - 3*w*t);
        c[1,14] := sin(2*w_sw*t - 3*w*t);
        c[1,15] := cos(2*w_sw*t + 3*w*t);
        c[1,16] := sin(2*w_sw*t + 3*w*t);
      end if;
    end Func_c;
    function Func_Ohm "Calculation of Matrix Ohm"
      extends Modelica.Icons.Function;
      input Integer Conf;
      input Real w_sw;
      input Real w;
      output Real[Conf, Conf] Ohm;
    algorithm
      Ohm := zeros(Conf, Conf);
      if Conf == 2 then
        Ohm[1,2] := -w;
        Ohm[2,1] := w;
      end if;
      if Conf == 8 then
        Ohm[1,2] := -w;
        Ohm[2,1] := w;
        Ohm[3,4] := -w_sw;
        Ohm[4,3] := w_sw;
        Ohm[5,6] := -(w_sw - 2*w);
        Ohm[6,5] := w_sw - 2*w;
        Ohm[7,8] := -(w_sw + 2*w);
        Ohm[8,7] := w_sw + 2*w;
      end if;
      if Conf == 6 then
        Ohm[1,2] := -w;
        Ohm[2,1] := w;
        Ohm[3,4] := -(w_sw - 2*w);
        Ohm[4,3] := w_sw - 2*w;
        Ohm[5,6] := -(w_sw + 2*w);
        Ohm[6,5] := w_sw + 2*w;
      end if;
      if Conf == 14 then
        Ohm[1,2] := -w;
        Ohm[2,1] := w;
        Ohm[3,4] := -(w_sw - 2*w);
        Ohm[4,3] := w_sw - 2*w;
        Ohm[5,6] := -(w_sw + 2*w);
        Ohm[6,5] := w_sw + 2*w;
        Ohm[7,8] := -(2*w_sw - w);
        Ohm[8,7] := 2*w_sw - w;
        Ohm[9,10] := -(2*w_sw + w);
        Ohm[10,9] := 2*w_sw + w;
        Ohm[11,12] := -(2*w_sw - 3*w);
        Ohm[12,11] := 2*w_sw - 3*w;
        Ohm[13,14] := -(2*w_sw + 3*w);
        Ohm[14,13] := 2*w_sw + 3*w;
      end if;
      if Conf == 16 then
        Ohm[1,2] := -w;
        Ohm[2,1] := w;
        Ohm[3,4] := -w_sw;
        Ohm[4,3] := w_sw;
        Ohm[5,6] := -(w_sw - 2*w);
        Ohm[6,5] := w_sw - 2*w;
        Ohm[7,8] := -(w_sw + 2*w);
        Ohm[8,7] := w_sw + 2*w;
        Ohm[9,10] := -(2*w_sw - w);
        Ohm[10,9] := 2*w_sw - w;
        Ohm[11,12] := -(2*w_sw + w);
        Ohm[12,11] := 2*w_sw + w;
        Ohm[13,14] := -(2*w_sw - 3*w);
        Ohm[14,13] := 2*w_sw - 3*w;
        Ohm[15,16] := -(2*w_sw + 3*w);
        Ohm[16,15] := 2*w_sw + 3*w;
      end if;
    end Func_Ohm;
    function Func_q01c
      extends Modelica.Icons.Function;
      input Real m01c;
      output Real q01c;
    algorithm
      q01c := 1/2*m01c;
    end Func_q01c;

    function Func_q01s
      extends Modelica.Icons.Function;
      input Real m01s;
      output Real q01s;
    algorithm
      q01s := 1/2*m01s;
    end Func_q01s;

    function Func_qnic
      extends Modelica.Icons.Function;
      import Modelica.Constants.pi;
      input Integer n;
      input Integer i;
      input Real m01c;
      input Real m01s;
      input Real fai;
      Real y_n;
      Real fai_m;
      output Real qnic;
    algorithm
      y_n := n*pi/2*sqrt(m01c^2 + m01s^2);
      fai_m := Modelica.Math.atan2(-m01s, m01c);
      qnic := 2/(n*pi)*sin(pi*(n + i)/2)*GNU_ScientificLibrary.Functions.specfunc.bessel_Jn(i, y_n)*cos(n*fai + i*fai_m);
    end Func_qnic;

    function Func_qnis
      extends Modelica.Icons.Function;
      import Modelica.Constants.pi;
      input Integer n;
      input Integer i;
      input Real m01c;
      input Real m01s;
      input Real fai;
      Real y_n;
      Real fai_m;
      output Real qnis;
    algorithm
      y_n := n*pi/2*sqrt(m01c^2 + m01s^2);
      fai_m := Modelica.Math.atan2(-m01s, m01c);
      qnis := -2/(n*pi)*sin(pi*(n + i)/2)*GNU_ScientificLibrary.Functions.specfunc.bessel_Jn(i, y_n)*sin(n*fai + i*fai_m);
    end Func_qnis;
  end Functions;

  annotation(
    Icon(graphics = {Rectangle(lineColor = {200, 200, 200}, fillColor = {248, 248, 248}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(lineColor = {128, 128, 128}, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(origin = {20.3125, 82.8571}, extent = {{-45.3125, -57.8571}, {4.6875, -27.8571}}), Line(origin = {8, 48}, points = {{32, -58}, {72, -58}}), Line(origin = {9, 54}, points = {{31, -49}, {71, -49}}), Line(origin = {-2, 55}, points = {{-83, -50}, {-33, -50}}), Line(origin = {-3, 45}, points = {{-72, -55}, {-42, -55}}), Line(origin = {1, 50}, points = {{-61, -45}, {-61, -10}, {-26, -10}}), Line(origin = {7, 50}, points = {{18, -10}, {53, -10}, {53, -45}}), Line(origin = {6.2593, 48}, points = {{53.7407, -58}, {53.7407, -93}, {-66.2593, -93}, {-66.2593, -58}}), Text(origin = {1, -71}, extent = {{-77, 29}, {77, -29}}, textString = "GAM", textStyle = {TextStyle.Bold})}), 
    uses(Modelica(version = "4.0.0")));
end PowerGAM;