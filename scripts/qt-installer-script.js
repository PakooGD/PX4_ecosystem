function Controller() {
    installer.autoRejectMessageBoxes();
    installer.installationFinished.connect(function() {
        gui.clickButton(buttons.NextButton);
    });
}

Controller.prototype.WelcomePageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.CredentialsPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.IntroductionPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.TargetDirectoryPageCallback = function() {
    gui.currentPageWidget().TargetDirectoryLineEdit.setText("/home/$USER/Qt");
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.ComponentSelectionPageCallback = function() {
    var widget = gui.currentPageWidget();

    // Включаем отображение архивных версий
    widget.showArchiveCheckBox.setChecked(true);
    widget.applyFilter();

    // Выбираем Qt 6.8.2
    widget.deselectAll();
    widget.selectComponent("qt.qt6.682");

    // Выбираем компоненты для Linux
    widget.selectComponent("qt.qt6.682.gcc_64"); // Desktop gcc 64-bit
    widget.selectComponent("qt.qt6.682.android"); // Android (опционально)

    // Выбираем все дополнительные библиотеки
    var additionalLibraries = widget.treeWidget.findItems("Additional Libraries", Qt.MatchContains);
    for (var i = 0; i < additionalLibraries.length; i++) {
        additionalLibraries[i].setChecked(0, true);
    }

    // Отключаем QT Design Studio
    var designStudio = widget.treeWidget.findItems("QT Design Studio", Qt.MatchExactly);
    for (var i = 0; i < designStudio.length; i++) {
        designStudio[i].setChecked(0, false);
    }

    gui.clickButton(buttons.NextButton);
}

Controller.prototype.LicenseAgreementPageCallback = function() {
    gui.currentPageWidget().AcceptLicenseRadioButton.setChecked(true);
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.StartMenuDirectoryPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.ReadyForInstallationPageCallback = function() {
    gui.clickButton(buttons.NextButton);
}

Controller.prototype.FinishedPageCallback = function() {
    gui.clickButton(buttons.FinishButton);
}