import os
import subprocess
import sys
import signal
import json
import multiprocessing
import threading
import time
import importlib.util

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
    QPushButton, QListWidget, QCheckBox, QLabel, QFileDialog, QMessageBox, QWidget,
    QDialog, QLineEdit
)
from PyQt5.QtCore import QProcess

from PyQt5.QtGui import QFont

from PyQt5.QtCore import Qt, QTimer, QDir

from launch_ros.actions import Node

CONFIG_FILE = "launch_config.json"


class Ros2LaunchManager:
    def __init__(self):
        self.launch_service = None  # Placeholder for LaunchService
        self.running_flag = multiprocessing.Value('b', False)
        self.launch_process = None

    def _run_launch_service(self, launch_description, arguments):
        """Führt den LaunchService aus (im separaten Prozess)."""
        from launch import LaunchService
        self.launch_service = LaunchService()
        self.launch_service.include_launch_description(launch_description)
        for key, value in arguments.items():
            self.launch_service.context.launch_configurations[key] = value
        self.running_flag.value = True
        self.launch_service.run()
        self.running_flag.value = False

    def start_launch(self, launch_description, arguments=None):
        """Startet den Launch im separaten Prozess."""
        if self.running_flag.value:
            print("Launch-Service läuft bereits.")
            return

        self.launch_process = multiprocessing.Process(
            target=self._run_launch_service, args=(launch_description, arguments)
        )
        self.launch_process.start()
        print("Launch gestartet.")

    def stop_launch(self):
        """Stoppt den Launch-Service."""
        if self.running_flag.value and self.launch_process:
            print("Stoppe den Launch...")
            os.kill(self.launch_process.pid, signal.SIGINT)
            self.launch_process.join()
            self.running_flag.value = False
            print("Launch gestoppt.")
        else:
            print("Kein laufender Launch-Service oder Prozess gefunden.")

class CustomLineEdit(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.keyboard_process = None

    def focusInEvent(self, event):
        """Wird aufgerufen, wenn das QLineEdit fokussiert wird."""
        super().focusInEvent(event)  
        self.show_keyboard()

    def show_keyboard(self):
        """Startet die On-Screen-Tastatur, falls sie nicht läuft."""
        if self.keyboard_process is None or self.keyboard_process.state() == QProcess.ProcessState.NotRunning:
            self.keyboard_process = QProcess(self)
            self.keyboard_process.start("onboard")

class LaunchFileDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Launch-Datei hinzufügen")
        self.setMinimumWidth(400)

        # Layouts
        main_layout = QVBoxLayout()

        # File Path Input
        file_layout = QHBoxLayout()
        self.file_path_label = QLabel("Dateipfad:")
        self.file_path_input = QLineEdit()
        self.browse_button = QPushButton("Durchsuchen")
        self.browse_button.clicked.connect(self.browse_file)
        file_layout.addWidget(self.file_path_label)
        file_layout.addWidget(self.file_path_input)
        file_layout.addWidget(self.browse_button)
        main_layout.addLayout(file_layout)

        # Description Input
        description_layout = QHBoxLayout()
        self.description_label = QLabel("Beschreibung:")
        self.description_input = CustomLineEdit()#QLineEdit()
        
        description_layout.addWidget(self.description_label)
        description_layout.addWidget(self.description_input)
        main_layout.addLayout(description_layout)

        # Launch Arguments Input
        arguments_layout = QHBoxLayout()
        self.arguments_label = QLabel("Argumente:")
        self.arguments_input = QLineEdit()
        self.arguments_input.setPlaceholderText("key1:=value1 key2:=value2 ...")
        arguments_layout.addWidget(self.arguments_label)
        arguments_layout.addWidget(self.arguments_input)
        main_layout.addLayout(arguments_layout)

        # Buttons
        button_layout = QHBoxLayout()
        self.ok_button = QPushButton("OK")
        self.ok_button.clicked.connect(self.accept)
        self.cancel_button = QPushButton("Abbrechen")
        self.cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(self.ok_button)
        button_layout.addWidget(self.cancel_button)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

    def browse_file(self):
        """Öffnet einen Datei-Dialog, um eine Datei auszuwählen."""
        qfiledialog = QFileDialog(self)
        qfiledialog.setFileMode(QFileDialog.FileMode.ExistingFile)
        qfiledialog.setOption(QFileDialog.Option.ShowDirsOnly, False)
        qfiledialog.setOption(QFileDialog.Option.DontUseNativeDialog, True)
        qfiledialog.setFilter(qfiledialog.filter() | QDir.Filter.Hidden)
        qfiledialog.setNameFilters(["Launch-Dateien (*.launch.py)", "Alle Dateien (*.*)"])
        if qfiledialog.exec():
            file_paths = qfiledialog.selectedFiles()
            if file_paths:
                self.file_path_input.setText(file_paths[0])

    def get_details(self):
        """Gibt die Details aus den Eingabefeldern zurück."""
        return {
            "file_path": self.file_path_input.text().strip(),
            "description": self.description_input.text().strip(),
            "arguments": self.arguments_input.text().strip()
        }

    def accept(self):
        """Überprüft die Eingaben und schließt den Dialog bei Erfolg."""
        if not self.file_path_input.text().strip():
            QMessageBox.warning(self, "Fehler", "Bitte geben Sie einen Dateipfad an.")
            return

        if not self.description_input.text().strip():
            QMessageBox.warning(self, "Fehler", "Bitte geben Sie eine Beschreibung ein.")
            return

        super().accept()


class LaunchApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ROS2 Launch Controller")
        self.resize(600, 400)

        self.launch_handlers = {}
        self.autostart = False
        self.countdown_active = False

        # Hauptlayout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # GUI-Elemente
        self.list_widget = QListWidget()
        # Schriftgröße anpassen
        font = QFont()
        font.setPointSize(20)
        self.list_widget.setFont(font)
        self.list_widget.setSelectionMode(QListWidget.MultiSelection)
        layout.addWidget(self.list_widget)

        button_layout = QHBoxLayout()
        self.add_button = QPushButton("Launch-Datei +")
        self.add_button.clicked.connect(self.add_launch_file)
        button_layout.addWidget(self.add_button)

        self.delete_button = QPushButton("Launch-Datei -")
        self.delete_button.clicked.connect(self.delete_selected)
        button_layout.addWidget(self.delete_button)

        self.start_button = QPushButton("Starten")
        self.start_button.clicked.connect(self.start_selected)
        button_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stoppen")
        self.stop_button.clicked.connect(self.stop_all)
        button_layout.addWidget(self.stop_button)

        self.killall_button = QPushButton("KillAll")
        self.killall_button.clicked.connect(self.killall_python)
        button_layout.addWidget(self.killall_button)

        layout.addLayout(button_layout)

        autostart_layout = QHBoxLayout()
        self.autostart_checkbox = QCheckBox("Autostart aktivieren")
        self.autostart_checkbox.stateChanged.connect(self.set_autostart)
        autostart_layout.addWidget(self.autostart_checkbox)

        self.countdown_label = QLabel("")
        autostart_layout.addWidget(self.countdown_label)

        self.abort_button = QPushButton("Autostart abbrechen")
        self.abort_button.setEnabled(False)
        self.abort_button.clicked.connect(self.abort_countdown)
        autostart_layout.addWidget(self.abort_button)
        layout.addLayout(autostart_layout)

        # Konfiguration laden und Countdown starten
        self.load_config()
        self.handle_autostart()

    def add_launch_file(self):
        """Öffnet den Dialog und verarbeitet die Eingaben."""
        dialog = LaunchFileDialog(self) 
        if dialog.exec() == QDialog.Accepted:
            details = dialog.get_details()
            file_path = details["file_path"]
            if file_path and file_path not in self.launch_handlers:
                key = details["description"] if details["description"] else file_path
                self.list_widget.addItem(key)
                self.launch_handlers[key] = (Ros2LaunchManager(), file_path, details["description"], details["arguments"].split())

    def delete_selected(self):
        """Ausgewählte Launch-Dateien starten."""
        selected_items = self.list_widget.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Warnung", "Bitte wähle mindestens eine Launch-Datei aus.")
            return

        for item in selected_items:
            file_path_or_description = item.text()
            self.launch_handlers.pop(file_path_or_description)

        for item in list(self.list_widget.selectedItems()):
            self.list_widget.takeItem(self.list_widget.row(item))

    def _load_launch_description(self, file_path):
        """Lädt die Launch-Datei."""
        spec = importlib.util.spec_from_file_location("launch_file", file_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.generate_launch_description()

    def _start_launch_file(self, file_path_or_description):
        try:
            manager, file_path, description, arguments = self.launch_handlers.get(file_path_or_description, (None, None, None, None))
            launch_description = self._load_launch_description(file_path)

            argument_overrides = {arg.split(":=")[0]: arg.split(":=")[1] for arg in arguments}

            # # LaunchService mit den Argumenten starten
            # manager.launch_service.include_launch_description(
            #     launch_description,
            #     launch_arguments=argument_overrides,
            # )

            manager.start_launch(launch_description, argument_overrides)
            
        except Exception as e:
            QMessageBox.critical(self, "Fehler", f"Fehler beim Starten der Launch-Datei: {e}")

    def start_selected(self):
        """Ausgewählte Launch-Dateien starten."""
        selected_items = self.list_widget.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Warnung", "Bitte wähle mindestens eine Launch-Datei aus.")
            return

        for item in selected_items:
            file_path_or_description = item.text()
            self._start_launch_file(file_path_or_description)
        QMessageBox.information(self, "Info", "Ausgewählte Launch-Dateien gestartet.")

    def stop_all(self):
        """Alle gestarteten Launch-Dateien stoppen."""
        for handler, file_path, description, arguments in self.launch_handlers.values():
            handler.stop_launch()
        QMessageBox.information(self, "Info", "Alle Launch-Dateien wurden gestoppt.")

    def killall_python(self):
        # pgrep -af python
        # Eigene PID abrufen
        current_pid = os.getpid()
        # Alle Python-Prozesse abrufen (außer sich selbst)
        #processes = subprocess.check_output(["pgrep", "-f", "python"]).decode().split()
        processes = subprocess.check_output(["pgrep", "-af", "python"]).decode().splitlines()
        for process in processes:
            args = process.split()
            pid = args[0]
            if int(pid) != current_pid:
                print(f"Killing PID {pid}")
                os.system(f"kill -9 {pid}")

    def set_autostart(self):
        """Schaltet den Autostart-Status um."""
        self.autostart = self.autostart_checkbox.isChecked()
        # current_state = self.autostart_checkbox.isChecked()
        # self.autostart_checkbox.setChecked(not current_state)

    def save_config(self):
        """Konfiguration speichern."""
        data = {
            "launch_files": [
                {
                    "description": self.launch_handlers[item.text()][2],
                    "file_path": self.launch_handlers[item.text()][1],
                    "arguments": self.launch_handlers[item.text()][3]
                }
                for item in [self.list_widget.item(i) for i in range(self.list_widget.count())]
            ],
            "autostart": self.autostart,
            "selected_indices": [i.row() for i in self.list_widget.selectedIndexes()]
        }
        with open(CONFIG_FILE, "w") as f:
            json.dump(data, f)

    def load_config(self):
        """Konfiguration laden."""
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
            for entry in data.get("launch_files", []):

                description = entry["description"]
                file_path = entry["file_path"]
                arguments = entry.get("arguments", [])

                key = description if description else file_path
                self.list_widget.addItem(key)
                self.launch_handlers[key] = (Ros2LaunchManager(), file_path, description, arguments)

            self.autostart = data.get("autostart", False)
            self.autostart_checkbox.setChecked(self.autostart)

            for index in data.get("selected_indices", []):
                self.list_widget.item(index).setSelected(True)

        except FileNotFoundError:
            pass

    def handle_autostart(self):
        """Autostart-Logik."""
        if self.autostart:
            self.countdown_active = True
            self.abort_button.setEnabled(True)

            def countdown():
                for i in range(5, 0, -1):
                    if not self.countdown_active:
                        self.countdown_label.setText("Autostart abgebrochen.")
                        self.abort_button.setEnabled(False)
                        return
                    self.countdown_label.setText(f"Autostart in {i} Sekunden...")
                    time.sleep(1)

                if self.countdown_active:
                    self.start_selected()
                self.countdown_label.setText("")
                self.abort_button.setEnabled(False)
                self.countdown_active = False

            threading.Thread(target=countdown, daemon=True).start()

    def abort_countdown(self):
        """Countdown abbrechen."""
        self.countdown_active = False

    def closeEvent(self, event):
        """Konfiguration speichern beim Schließen."""
        self.save_config()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LaunchApp()
    window.show()
    sys.exit(app.exec_())
