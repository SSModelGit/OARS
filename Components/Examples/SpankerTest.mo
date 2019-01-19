within OARS.Components.Examples;

model SpankerTest
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(visible = true, transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Body attached to the spanker
  // Constant input
  Modelica.Blocks.Sources.Constant xcomp(k = 1) annotation(Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant ycomp(k = 0) annotation(Placement(visible = true, transformation(origin = {-50, -35.609}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant zcomp(k = 0) annotation(Placement(visible = true, transformation(origin = {-50, -76.09}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = Modelica.Constants.pi / 180) annotation(Placement(visible = true, transformation(origin = {-50, 37.054}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Real k_dw = 1000;
  // spanker
  Instances.Spanker spanker(r_COL = {0, 0, 0}, heightDirection = {0, 1, 0}, height = 1, width = 0.01, r_0.fixed = true, v_0.fixed = true, r_0.start = {0, 0, 0}, v_0.start = {0, 0, 0}, color = {0, 0, 0}, m = 10, length = 0.6) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Sensor
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity sail_v(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(Placement(visible = true, transformation(origin = {25.985, -27.609}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity sail_w(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) "Absolute angular velocity of the boat" annotation(Placement(visible = true, transformation(origin = {25.985, -48.376}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Wind velocity vector
  Real v_w_W[3] "Wind absolute velocity vector resolved in the world frame";
  // Raft
  Modelica.Mechanics.MultiBody.Parts.BodyBox bodyBox1(r = {0.5, 0, 0}, r_shape = {-0.5, 0, 0}, widthDirection = {0, 0, 1}, width = 1, height = 0.01, r_0.fixed = false, v_0.fixed = false, angles_fixed = false, w_0_fixed = false, z_0_fixed = false, a_0.fixed = false) annotation(Placement(visible = true, transformation(origin = {13.248, -65.94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Damping
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque angDamping(animation = false) annotation(Placement(visible = true, transformation(origin = {-15.128, -82.66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Sine sine1(freqHz = 1, amplitude = 45, offset = 1) annotation(Placement(visible = true, transformation(origin = {-82.014, 37.353}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  // Determing relative velocity
  v_w_W = {xcomp.y, ycomp.y, zcomp.y};
  spanker.windVel = v_w_W - sail_v.v;
  // Damping
  angDamping.torque[1] = -k_dw * sail_w.w[1];
  angDamping.torque[2] = 0;
  angDamping.torque[3] = -k_dw * sail_w.w[3];
  angDamping.force = zeros(3);
  connect(spanker.frame_a, sail_v.frame_a) annotation(Line(visible = true, origin = {5.328, -21.739}, points = {{-5.328, 11.739}, {-5.328, -5.87}, {10.657, -5.87}}, color = {95, 95, 95}));
  connect(gain1.y, spanker.heading) annotation(Line(visible = true, origin = {-20.25, 14.827}, points = {{-18.75, 22.227}, {5.25, 22.227}, {5.25, -22.227}, {8.25, -22.227}}, color = {1, 37, 163}));
  connect(bodyBox1.frame_a, spanker.frame_a) annotation(Line(visible = true, origin = {1.083, -47.293}, points = {{2.165, -18.647}, {-1.083, -18.647}, {-1.083, 37.293}}, color = {95, 95, 95}));
  connect(sail_w.frame_a, spanker.frame_a) annotation(Line(visible = true, origin = {5.328, -35.584}, points = {{10.657, -12.792}, {-5.328, -12.792}, {-5.328, 25.584}}, color = {95, 95, 95}));
  connect(angDamping.frame_b, bodyBox1.frame_a) annotation(Line(visible = true, origin = {-0.458, -74.3}, points = {{-4.669, -8.36}, {0.481, -8.36}, {0.481, 8.36}, {3.706, 8.36}}, color = {95, 95, 95}));
  connect(sine1.y, gain1.u) annotation(Line(visible = true, origin = {-65.754, 37.203}, points = {{-5.261, 0.149}, {0.754, 0.149}, {0.754, -0.149}, {3.754, -0.149}}, color = {1, 37, 163}));
end SpankerTest;
