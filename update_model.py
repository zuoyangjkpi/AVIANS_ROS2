from pathlib import Path
path = Path('src/drone_description/models/x3/model.sdf')
text = path.read_text()
marker = '    <!-- VelocityControl removed - using MulticopterVelocityControl from world file instead -->\n\n'
plugin = "    <!-- Odometry publisher plugin -->\n    <plugin filename=\"gz-sim8-odometry-publisher-system\" name=\"gz::sim::systems::OdometryPublisher\">\n      <odom_frame>X3/odometry</odom_frame>\n      <robot_base_frame>X3/base_link</robot_base_frame>\n      <odom_topic>/X3/odometry</odom_topic>\n      <odom_publish_frequency>50</odom_publish_frequency>\n    </plugin>\n\n"
if marker not in text:
    raise SystemExit('marker not found')
text = text.replace(marker, marker + plugin)
path.write_text(text)
