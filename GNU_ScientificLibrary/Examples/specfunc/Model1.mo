model Model1
  annotation(Diagram(coordinateSystem(extent={{-100,-100},{100,100}}, 
grid={2,2})));
  Modelica.Blocks.Sources.Ramp ramp(duration = 20, height = 20) annotation(
    Placement(transformation(origin={6,0}, 
extent={{-80,-10},{-60,10}})));
  Blocks.specfunc.Bessel_J0 bessel_J0 annotation(
    Placement(transformation(origin={-16,0}, 
extent={{-10,-10},{10,10}})));
equation
  connect(ramp.y, bessel_J0.x) 
  annotation(Line(origin={-40,0}, 
  points={{-13,0},{12,0}}, 
  color={0,0,127}));

end Model1;