# labyrinth_turtlebot

## Requirements

- turtlebot3
- turtlebot3_msgs
- turtlebot3_simulations
- gazebo
- rviz

rostopic pub /cmd_vel geometry_msgs/Twist "linear:
x: 0.5
y: 0.0
z: 0.0
angular:
x: 0.0
y: 0.0
z: 0.0" -r 3

## Problem mit den Markern:

**Vermutung**: Alte Marker vorhanden, da neuer Weg kuerzer, bei gleicher max. Array Laenge -> alte letzte Wegpunkte noch da
**Lösung**: Clear Marker Array beforehand, set new max size to path length

## Problem mit Wegfindung:

**Vermutung**: Kann keine neuen Wegpunkte finden, da kein natürlicher Map Weg existiert zu dem Zeitpunkt
**Lösung**  : Erlaube die Verwendung imaginärer Mappunkte wie z.B. -1, 0 oder ähnliches.

## Problem mit new Map

**Vermutung**: Bei Map update verschiebt sich der Map Origin, dadurch können gespeicherte Punkte nicht wiederverwendet werden
**Lösung**: Resette alle Punkte
