/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *act_new;
    QWidget *centralWidget;
    QLabel *label_hello;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *Button_open;
    QPushButton *Button_zero;
    QPushButton *Button_close;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout;
    QRadioButton *Button_red;
    QRadioButton *Button_blue;
    QRadioButton *Button_black;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *checkBox_underline;
    QCheckBox *checkBox_bold;
    QSpacerItem *horizontalSpacer;
    QCheckBox *checkBox_italic;
    QPlainTextEdit *Text_helloqt;
    QPushButton *pushButton_Ok;
    QPushButton *pushButton_Cancel;
    QMenuBar *menuBar;
    QMenu *menu;
    QMenu *menu_2;
    QMenu *menu_3;
    QMenu *menu_4;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(592, 341);
        QFont font;
        font.setFamily(QString::fromUtf8("Adobe \345\256\213\344\275\223 Std L"));
        font.setPointSize(16);
        MainWindow->setFont(font);
        act_new = new QAction(MainWindow);
        act_new->setObjectName(QStringLiteral("act_new"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        label_hello = new QLabel(centralWidget);
        label_hello->setObjectName(QStringLiteral("label_hello"));
        label_hello->setGeometry(QRect(6, 136, 40, 16));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(395, 6, 82, 76));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        Button_open = new QPushButton(verticalLayoutWidget);
        Button_open->setObjectName(QStringLiteral("Button_open"));
        Button_open->setCheckable(false);

        verticalLayout->addWidget(Button_open);

        Button_zero = new QPushButton(verticalLayoutWidget);
        Button_zero->setObjectName(QStringLiteral("Button_zero"));

        verticalLayout->addWidget(Button_zero);

        Button_close = new QPushButton(verticalLayoutWidget);
        Button_close->setObjectName(QStringLiteral("Button_close"));

        verticalLayout->addWidget(Button_close);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 160, 581, 59));
        horizontalLayout = new QHBoxLayout(groupBox);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        Button_red = new QRadioButton(groupBox);
        Button_red->setObjectName(QStringLiteral("Button_red"));

        horizontalLayout->addWidget(Button_red);

        Button_blue = new QRadioButton(groupBox);
        Button_blue->setObjectName(QStringLiteral("Button_blue"));

        horizontalLayout->addWidget(Button_blue);

        Button_black = new QRadioButton(groupBox);
        Button_black->setObjectName(QStringLiteral("Button_black"));

        horizontalLayout->addWidget(Button_black);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(0, 200, 491, 91));
        horizontalLayout_2 = new QHBoxLayout(groupBox_2);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        checkBox_underline = new QCheckBox(groupBox_2);
        checkBox_underline->setObjectName(QStringLiteral("checkBox_underline"));

        horizontalLayout_2->addWidget(checkBox_underline);

        checkBox_bold = new QCheckBox(groupBox_2);
        checkBox_bold->setObjectName(QStringLiteral("checkBox_bold"));

        horizontalLayout_2->addWidget(checkBox_bold);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        checkBox_italic = new QCheckBox(groupBox_2);
        checkBox_italic->setObjectName(QStringLiteral("checkBox_italic"));

        horizontalLayout_2->addWidget(checkBox_italic);

        Text_helloqt = new QPlainTextEdit(centralWidget);
        Text_helloqt->setObjectName(QStringLiteral("Text_helloqt"));
        Text_helloqt->setGeometry(QRect(6, 6, 385, 126));
        pushButton_Ok = new QPushButton(centralWidget);
        pushButton_Ok->setObjectName(QStringLiteral("pushButton_Ok"));
        pushButton_Ok->setGeometry(QRect(6, 282, 80, 22));
        pushButton_Cancel = new QPushButton(centralWidget);
        pushButton_Cancel->setObjectName(QStringLiteral("pushButton_Cancel"));
        pushButton_Cancel->setGeometry(QRect(201, 282, 80, 22));
        MainWindow->setCentralWidget(centralWidget);
        groupBox->raise();
        label_hello->raise();
        verticalLayoutWidget->raise();
        groupBox_2->raise();
        Text_helloqt->raise();
        pushButton_Ok->raise();
        pushButton_Cancel->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 592, 21));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        menu_2 = new QMenu(menuBar);
        menu_2->setObjectName(QStringLiteral("menu_2"));
        menu_3 = new QMenu(menuBar);
        menu_3->setObjectName(QStringLiteral("menu_3"));
        menu_4 = new QMenu(menuBar);
        menu_4->setObjectName(QStringLiteral("menu_4"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);

        menuBar->addAction(menu->menuAction());
        menuBar->addAction(menu_2->menuAction());
        menuBar->addAction(menu_3->menuAction());
        menuBar->addAction(menu_4->menuAction());
        menu->addAction(act_new);

        retranslateUi(MainWindow);
        QObject::connect(Button_close, SIGNAL(clicked()), MainWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        act_new->setText(QApplication::translate("MainWindow", "\346\226\260\345\273\272", Q_NULLPTR));
#ifndef QT_NO_SHORTCUT
        act_new->setShortcut(QApplication::translate("MainWindow", "Ctrl+N", Q_NULLPTR));
#endif // QT_NO_SHORTCUT
        label_hello->setText(QApplication::translate("MainWindow", "Hello!", Q_NULLPTR));
        Button_open->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200", Q_NULLPTR));
        Button_zero->setText(QApplication::translate("MainWindow", "\347\275\256\351\233\266", Q_NULLPTR));
        Button_close->setText(QApplication::translate("MainWindow", "\345\205\263\351\227\255", Q_NULLPTR));
        groupBox->setTitle(QString());
        Button_red->setText(QApplication::translate("MainWindow", "red", Q_NULLPTR));
        Button_blue->setText(QApplication::translate("MainWindow", "blue", Q_NULLPTR));
        Button_black->setText(QApplication::translate("MainWindow", "blue", Q_NULLPTR));
        groupBox_2->setTitle(QString());
        checkBox_underline->setText(QApplication::translate("MainWindow", "underline", Q_NULLPTR));
        checkBox_bold->setText(QApplication::translate("MainWindow", "Bold", Q_NULLPTR));
        checkBox_italic->setText(QApplication::translate("MainWindow", "Italic", Q_NULLPTR));
        Text_helloqt->setPlainText(QApplication::translate("MainWindow", "Hello world!\n"
"Hello qt!", Q_NULLPTR));
        pushButton_Ok->setText(QApplication::translate("MainWindow", "OK", Q_NULLPTR));
        pushButton_Cancel->setText(QApplication::translate("MainWindow", "Cancel", Q_NULLPTR));
        menu->setTitle(QApplication::translate("MainWindow", "\346\226\207\344\273\266", Q_NULLPTR));
        menu_2->setTitle(QApplication::translate("MainWindow", "\347\274\226\350\276\221", Q_NULLPTR));
        menu_3->setTitle(QApplication::translate("MainWindow", "\350\247\206\345\233\276", Q_NULLPTR));
        menu_4->setTitle(QApplication::translate("MainWindow", "\345\267\245\345\205\267", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
