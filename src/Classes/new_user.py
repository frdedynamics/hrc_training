# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/gizem/catkin_ws/src/hrc_training/ui/new_user.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(409, 213)
        Dialog.setModal(False)
        self.nameLabel = QtWidgets.QLabel(Dialog)
        self.nameLabel.setGeometry(QtCore.QRect(30, 30, 61, 17))
        self.nameLabel.setObjectName("nameLabel")
        self.heightLabel = QtWidgets.QLabel(Dialog)
        self.heightLabel.setGeometry(QtCore.QRect(30, 70, 61, 17))
        self.heightLabel.setObjectName("heightLabel")
        self.leftHandcheckBox = QtWidgets.QCheckBox(Dialog)
        self.leftHandcheckBox.setGeometry(QtCore.QRect(290, 70, 121, 23))
        self.leftHandcheckBox.setObjectName("leftHandcheckBox")
        self.armLengthLabel = QtWidgets.QLabel(Dialog)
        self.armLengthLabel.setGeometry(QtCore.QRect(30, 110, 91, 17))
        self.armLengthLabel.setObjectName("armLengthLabel")
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(120, 160, 166, 25))
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.nameLineEdit = QtWidgets.QLineEdit(Dialog)
        self.nameLineEdit.setGeometry(QtCore.QRect(120, 20, 131, 25))
        self.nameLineEdit.setObjectName("nameLineEdit")
        self.armLengthLineEdit = QtWidgets.QLineEdit(Dialog)
        self.armLengthLineEdit.setGeometry(QtCore.QRect(120, 100, 131, 25))
        self.armLengthLineEdit.setObjectName("armLengthLineEdit")
        self.heightLineEdit = QtWidgets.QLineEdit(Dialog)
        self.heightLineEdit.setGeometry(QtCore.QRect(120, 60, 131, 25))
        self.heightLineEdit.setObjectName("heightLineEdit")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "New User"))
        self.nameLabel.setText(_translate("Dialog", "Name:"))
        self.heightLabel.setText(_translate("Dialog", "Height:"))
        self.leftHandcheckBox.setText(_translate("Dialog", "Left Handed"))
        self.armLengthLabel.setText(_translate("Dialog", "Arm Lenght:"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())
