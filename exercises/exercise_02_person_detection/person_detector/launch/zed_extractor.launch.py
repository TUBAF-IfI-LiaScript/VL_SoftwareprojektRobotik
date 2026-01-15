"""
Launch file für den ZED-Extraktor Node.

AUFGABE 1: Erstellen Sie ein Launch-File für den ZED-Extraktor Node.

Verwendung (nach Implementierung):
    ros2 launch person_detector zed_extractor.launch.py
    ros2 launch person_detector zed_extractor.launch.py output_csv:=results/my_zed.csv

Hinweise:
    - Nutzen Sie DeclareLaunchArgument für konfigurierbare Parameter
    - LaunchConfiguration ermöglicht den Zugriff auf Launch-Argumente
    - Der Node benötigt den Parameter 'output_csv' für die Ausgabedatei
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # =========================================================================
    # TODO 1: Launch-Argument für die Ausgabedatei deklarieren
    # =========================================================================
    # Erstellen Sie ein DeclareLaunchArgument mit:
    #   - Name: 'output_csv'
    #   - Default-Wert: 'results/zed_detections.csv'
    #   - Beschreibung: 'Output CSV file path for ZED detections'
    #
    # Beispiel-Syntax:
    #   my_arg = DeclareLaunchArgument(
    #       'parameter_name',
    #       default_value='default_value',
    #       description='Description of the parameter'
    #   )
    # -------------------------------------------------------------------------
    output_arg = None  # TODO: Ersetzen Sie None durch DeclareLaunchArgument(...)

    # =========================================================================
    # TODO 2: ZED Extractor Node erstellen
    # =========================================================================
    # Erstellen Sie einen Node mit:
    #   - package: 'person_detector'
    #   - executable: 'zed_extractor_node'
    #   - name: 'zed_extractor'
    #   - parameters: Dictionary mit 'output_csv' aus LaunchConfiguration
    #   - output: 'screen'
    #   - emulate_tty: True (für farbige Ausgabe)
    #
    # Beispiel für Parameter mit LaunchConfiguration:
    #   parameters=[{
    #       'my_param': LaunchConfiguration('my_arg_name'),
    #   }]
    # -------------------------------------------------------------------------
    extractor_node = None  # TODO: Ersetzen Sie None durch Node(...)

    # =========================================================================
    # TODO 3: LaunchDescription zurückgeben
    # =========================================================================
    # Fügen Sie output_arg und extractor_node zur Liste hinzu
    # -------------------------------------------------------------------------
    return LaunchDescription([
        # TODO: Fügen Sie hier die Launch-Elemente ein
    ])
