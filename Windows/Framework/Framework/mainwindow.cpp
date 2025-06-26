//
// (c) Georg Umlauf, 2021
//
#include "mainwindow.h"
#include <QFileDialog>
#include <QKeyEvent>
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindowClass)
{
    ui->setupUi(this);
    ui->glwidget->setFocusPolicy(Qt::StrongFocus);
    ui->glwidget->setFocus();

    connect(ui->pushButton, &QPushButton ::clicked, ui->glwidget, &GLWidget ::openFileDialog);
    connect(ui->radioButton_1, &QRadioButton::clicked, ui->glwidget, &GLWidget ::radioButtonClicked);
    connect(ui->radioButton_2, &QRadioButton::clicked, ui->glwidget, &GLWidget ::radioButtonClicked);
    connect(ui->horizontalSlider, &QSlider ::valueChanged, this, &MainWindow::updatePointSize);
    connect(ui->horizontalSliderBBoxDepth, &QSlider::valueChanged, this, &MainWindow::updateBBoxDepth);

    updatePointSize(3);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updatePointSize(int value)
{
    std::cout << "new pointsize: " << value << std::endl;
    ui->glwidget->setPointSize(value);
}

void MainWindow::updateBBoxDepth(int value)
{
    std::cout << "new bbox depth: " << value << std::endl;
    ui->glwidget->setBBoxDepth(value);
}
