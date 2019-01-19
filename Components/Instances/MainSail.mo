within OARS.Components.Instances;

model MainSail
  extends OARS.Components.Templates.SailWithMount(sailOrientation.n = {0, 1, 0}, servoJoint.n = {1, 0, 0}, servoJoint.useAxisFlange = true);
equation

  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {31.513, -12.549}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, points = {{-85.894, 67.767}, {32.907, 14.223}, {52.986, -81.989}})}));
end MainSail;
