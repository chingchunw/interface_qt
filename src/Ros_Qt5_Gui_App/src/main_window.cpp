/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui->
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include<QDebug>
#include <iostream>
#include "../include/cyrobot_monitor/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cyrobot_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent),
    ui(new Ui::MainWindowDesign), 
    qnode(argc,argv)
{
    ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    //QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    initUis();
    initData();
    //读取配置文件
    ReadSettings();
    setWindowIcon(QIcon(":/images/robot.png"));
    ui->tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    //QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    
    /*********************
    ** Logging
    **********************/
//    ui->view_logging->setModel(qnode.loggingModel());

    addtopic_form = new AddTopics();
    //绑定添加rviz话题信号
    connect(addtopic_form, SIGNAL(Topic_choose(QTreeWidgetItem *, QString)), this, SLOT(slot_choose_topic(QTreeWidgetItem *, QString)));
    
    /*********************
    ** 自动连接master
    **********************/
    if ( ui->checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked();
    }
    //链接connect
    connections();

}

//析构函数
MainWindow::~MainWindow()
{
    if( base_cmd)
    {
        delete base_cmd;
        base_cmd = nullptr;
    }
    if(map_rviz_)
    {
        delete map_rviz_;
        map_rviz_ = nullptr;
    }
}

//初始化UI
void MainWindow::initUis()
{
    ui->groupBox_3->setEnabled(false);

    ui->tab_manager->setCurrentIndex(0);
    ui->tabWidget->setCurrentIndex(0);
    
    ui->pushButton_remove_topic->setEnabled(false);
    ui->pushButton_rename_topic->setEnabled(false);    
    ui->button_disconnect->setEnabled(false);
}

void MainWindow::initData()
{
    m_mapRvizDisplays.insert("Axes", RVIZ_DISPLAY_AXES);
    m_mapRvizDisplays.insert("Camera", RVIZ_DISPLAY_CAMERA);
    m_mapRvizDisplays.insert("DepthCloud", RVIZ_DISPLAY_DEPTHCLOUD);
    m_mapRvizDisplays.insert("Effort", RVIZ_DISPLAY_EFFORT);
    m_mapRvizDisplays.insert("FluidPressure", RVIZ_DISPLAY_FLUIDPRESSURE);
    m_mapRvizDisplays.insert("Grid", RVIZ_DISPLAY_GRID);
    m_mapRvizDisplays.insert("GridCells", RVIZ_DISPLAY_GRIDCELLS);
    m_mapRvizDisplays.insert("Group", RVIZ_DISPLAY_GROUP);
    m_mapRvizDisplays.insert("Illuminance", RVIZ_DISPLAY_ILLUMINANCE);
    m_mapRvizDisplays.insert("Image", RVIZ_DISPLAY_IMAGE);
    m_mapRvizDisplays.insert("InterativerMarker", RVIZ_DISPLAY_INTERATIVEMARKER);
    m_mapRvizDisplays.insert("LaserScan", RVIZ_DISPLAY_LASERSCAN);
    m_mapRvizDisplays.insert("Map", RVIZ_DISPLAY_MAP);
    m_mapRvizDisplays.insert("Marker", RVIZ_DISPLAY_MARKER);
    m_mapRvizDisplays.insert("MarkerArray", RVIZ_DISPLAY_MARKERARRAY);
    m_mapRvizDisplays.insert("Odometry", RVIZ_DISPLAY_ODOMETRY);
    m_mapRvizDisplays.insert("Path", RVIZ_DISPLAY_PATH);
    m_mapRvizDisplays.insert("PointCloud", RVIZ_DISPLAY_POINTCLOUD);
    m_mapRvizDisplays.insert("PointCloud2", RVIZ_DISPLAY_POINTCLOUD2);
    m_mapRvizDisplays.insert("PointStamped", RVIZ_DISPLAY_POINTSTAMPED);
    m_mapRvizDisplays.insert("Polygon", RVIZ_DISPLAY_POLYGON);
    m_mapRvizDisplays.insert("Pose", RVIZ_DISPLAY_POSE);
    m_mapRvizDisplays.insert("PoseArray", RVIZ_DISPLAY_POSEARRAY);
    m_mapRvizDisplays.insert("PoseWithCovariance", RVIZ_DISPLAY_POSEWITHCOVARIANCE);
    m_mapRvizDisplays.insert("Range", RVIZ_DISPLAY_RANGE);
    m_mapRvizDisplays.insert("RelativeHumidity", RVIZ_DISPLAY_RELATIVEHUMIDITY);
    m_mapRvizDisplays.insert("RobotModel", RVIZ_DISPLAY_ROBOTMODEL);
    m_mapRvizDisplays.insert("TF", RVIZ_DISPLAY_TF);
    m_mapRvizDisplays.insert("Temperature", RVIZ_DISPLAY_TEMPERATURE);
    m_mapRvizDisplays.insert("WrenchStamped", RVIZ_DISPLAY_WRENCHSTAMPED);
}

void MainWindow::initRviz()
{
    ui->label_rvizShow->hide();
    map_rviz_=new QRviz(ui->verticalLayout_build_map,"qrviz");
    connect(map_rviz_, &QRviz::ReturnModelSignal, this, &MainWindow::RvizGetModel);
    map_rviz_->GetDisplayTreeModel();
    QMap<QString, QVariant> namevalue;
    namevalue.insert("Line Style", "Billboards");
    namevalue.insert("Color", QColor(160, 160, 160));
    namevalue.insert("Plane Cell Count", 10);
    map_rviz_->DisplayInit(RVIZ_DISPLAY_GRID, "Grid", true, namevalue);
    
    ui->pushButton_add_topic->setEnabled(true);
    ui->pushButton_rvizReadDisplaySet->setEnabled(true);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(true);
}

void MainWindow::RvizGetModel(QAbstractItemModel *model)
{
    m_modelRvizDisplay = model;
    ui->treeView_rvizDisplayTree->setModel(model);
}

void MainWindow::connections()
{
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(slot_rosShutdown()));
    QObject::connect(&qnode, SIGNAL(Master_shutdown()), this, SLOT(slot_rosShutdown()));
    //设置界面
    connect(ui->action_2,SIGNAL(triggered(bool)),this,SLOT(slot_setting_frame()));
    //设置2D Pose
    connect(ui->set_pos_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Pos()));
    //设置2D goal
    connect(ui->set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_2D_Goal()));
    //设置MoveCamera
    connect(ui->move_camera_btn,SIGNAL(clicked()),this,SLOT(slot_move_camera_btn()));
    //设置Select
    connect(ui->set_select_btn,SIGNAL(clicked()),this,SLOT(slot_set_select()));
    //刷新话题列表
    connect(ui->refreash_topic_btn,SIGNAL(clicked()),this,SLOT(refreashTopicList()));
}
//设置界面
void MainWindow::slot_setting_frame()
{
    if(set != nullptr)
    {
        delete set;
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    else
    {
        set=new Settings();
        set->setWindowModality(Qt::ApplicationModal);
        set->show();
    }
    //绑定set确认按钮点击事件
}
//设置导航当前位置按钮的槽函数
void MainWindow::slot_set_2D_Pos()
{
 map_rviz_->Set_Pos();
// ui->label_map_msg->setText("请在地图中选择机器人的初始位置");
}
//设置导航目标位置按钮的槽函数
void MainWindow::slot_set_2D_Goal()
{
  map_rviz_->Set_Goal();
}
void MainWindow::slot_move_camera_btn()
{
    map_rviz_->Set_MoveCamera();
    qDebug()<<"move camera";
}
void MainWindow::slot_set_select()
{
    map_rviz_->Set_Select();
}


//选中要添加的话题的槽函数
void MainWindow::slot_choose_topic(QTreeWidgetItem *choose, QString name)
{
    QString ClassID = choose->text(0);
    // 检查重名
    name = JudgeDisplayNewName(name);
    
    qDebug() << "choose topic ClassID:" << ClassID << ", name:" << name;
    QMap<QString, QVariant> namevalue;
    namevalue.clear();
    map_rviz_->DisplayInit(m_mapRvizDisplays[ClassID], name, true, namevalue);
}

//

QString MainWindow::JudgeDisplayNewName(QString name)
{
    if (m_modelRvizDisplay != nullptr)
    {
        bool same = true;
        while (same)
        {
            same = false;
            for (int i = 0; i < m_modelRvizDisplay->rowCount(); i++)
            {
                //m_sRvizDisplayChooseName = index.data().value<QString>();
                if (m_modelRvizDisplay->index(i, 0).data().value<QString>() == name)
                {
                    if (name.indexOf("_") != -1)
                    {
                        int num = name.section("_", -1, -1).toInt();
                        name = name.left(name.length() - name.section("_", -1, -1).length() - 1);
                        if (num <= 1)
                        {
                            num = 2;
                        }
                        else
                        {
                            num++;
                        }
                        name = name + "_" + QString::number(num);
                    }
                    else {
                      name = name + "_2";
                    }
                    same = true;
                    break;
                }
            }
        }
    }
    return name;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::inform(QString strdata)
{
    QMessageBox m_r;
    m_r.setWindowTitle("Hint");
    m_r.setText(strdata);
    m_r.exec();
}
bool MainWindow::AskInform(QString strdata)
{
    QMessageBox m_r;

    m_r.setWindowTitle("Hint");
    m_r.setText(strdata);
    m_r.addButton(tr("Confirm"), QMessageBox::ActionRole);
    m_r.addButton(tr("Cancel"), QMessageBox::ActionRole);

    int isok = m_r.exec();
    if (isok == 1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::initTopicList()
{
    ui->topic_listWidget->clear();
    ui->topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
       ui->topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
    }
}
void MainWindow::refreashTopicList()
{
    initTopicList();
}
//当ros与master的连接断开时
void MainWindow::slot_rosShutdown()
{
    ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
    ui->label_statue_text->setStyleSheet("color:red;");
    ui->label_statue_text->setText("Offline");
    ui->button_connect->setEnabled(true);
    ui->line_edit_master->setReadOnly(false);
    ui->line_edit_host->setReadOnly(false);
    ui->line_edit_topic->setReadOnly(false);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
//        ui->view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    //QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "cyrobot_rviz_tree");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui->line_edit_master->setText(master_url);
    ui->line_edit_host->setText(host_url);
    //ui->line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui->checkbox_remember_settings->setChecked(remember);

}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "cyrobot_rviz_tree");
    settings.setValue("master_url",ui->line_edit_master->text());
    settings.setValue("host_url",ui->line_edit_host->text());
    //settings.setValue("topic_name",ui->line_edit_topic->text());
    settings.setValue("geometry", saveGeometry());
    //settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui->checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace cyrobot_monitor

///
/// \brief 增加topic
///
void cyrobot_monitor::MainWindow::on_pushButton_add_topic_clicked()
{
    addtopic_form->show();
}

///
/// \brief 删除topic
///
void cyrobot_monitor::MainWindow::on_pushButton_remove_topic_clicked()
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
    {
        m_sRvizDisplayChooseName_ = ui->treeView_rvizDisplayTree->currentIndex().data().value<QString>();
        map_rviz_->RemoveDisplay(m_sRvizDisplayChooseName_);
        if (ui->treeView_rvizDisplayTree->currentIndex().row() >= 0)
        {
            on_treeView_rvizDisplayTree_clicked(ui->treeView_rvizDisplayTree->currentIndex());
        }
        else
        {
            m_sRvizDisplayChooseName_.clear();
        }
    }
    else
    {
        inform("请选择Display后再执行此操作");
    }
}

///
/// \brief 重命名topic
///
void cyrobot_monitor::MainWindow::on_pushButton_rename_topic_clicked()
{
    if (ui->treeView_rvizDisplayTree->currentIndex().row() < 0)
    {
        inform("请选择Display后再执行此操作");
        return ;
    }
    QString dlgTitle = "重命名";
    QString txtlabel = "请输入名字：";
    QString defaultInupt = m_sRvizDisplayChooseName_;
    QLineEdit::EchoMode echoMode = QLineEdit::Normal;
    bool ok = false;
    QString newname = QInputDialog::getText(this, dlgTitle, txtlabel, echoMode, defaultInupt, &ok);
    if (ok && !newname.isEmpty())
    {
        if (newname != defaultInupt)
        {
            QString nosamename = JudgeDisplayNewName(newname);
            map_rviz_->RenameDisplay(defaultInupt, nosamename);
            m_sRvizDisplayChooseName_ = nosamename;
            if (nosamename != newname)
            {
                inform("命名重复！命名已自动更改为" + nosamename);
            }
        }
    }
    else if (ok)
    {
        inform("输入内容为空，重命名失败");
    }
}

void cyrobot_monitor::MainWindow::on_treeView_rvizDisplayTree_clicked(const QModelIndex &index)
{
    m_sRvizDisplayChooseName_ = index.data().value<QString>();
    if (index.parent().row() == -1)   // Display 的根节点
    {
        if (index.row() > 1)
        {
            ui->pushButton_remove_topic->setEnabled(true);
            ui->pushButton_rename_topic->setEnabled(true);
        }
        else
        {
            ui->pushButton_remove_topic->setEnabled(false);
            ui->pushButton_rename_topic->setEnabled(false);
        }
    }
    else
    {
        ui->pushButton_remove_topic->setEnabled(false);
        ui->pushButton_rename_topic->setEnabled(false);
    }
}

/// \brief 连接ROS
void cyrobot_monitor::MainWindow::on_button_connect_clicked()
{
      if ( !qnode.init() )
      {
          //showNoMasterMessage();
          QMessageBox::warning(nullptr, "Error", "Connect to ROS Master failed！Please check your necwork！", QMessageBox::Yes, QMessageBox::Yes);
          ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/offline.png")));
          ui->label_statue_text->setStyleSheet("color:red;");
          ui->label_statue_text->setText("Offline");
          ui->tab_manager->setTabEnabled(1,false);
          ui->tabWidget->setTabEnabled(1,false);
          ui->groupBox_3->setEnabled(false);
          return ;
      }

    ui->tab_manager->setTabEnabled(1,true);
    ui->tabWidget->setTabEnabled(1,true);
    ui->groupBox_3->setEnabled(true);
    //初始化rviz
    initRviz();
    
    ui->button_connect->setEnabled(false);
    ui->label_robot_staue_img->setPixmap(QPixmap::fromImage(QImage("://images/online.png")));
    ui->label_statue_text->setStyleSheet("color:green;");
    ui->label_statue_text->setText("Online");
    ui->button_disconnect->setEnabled(true);
    //显示话题列表
    initTopicList();
}

/// \brief 断开ROS
void cyrobot_monitor::MainWindow::on_button_disconnect_clicked()
{
    qnode.disinit();
    map_rviz_->quit();
    delete map_rviz_;
    map_rviz_ = nullptr;
    ui->pushButton_add_topic->setEnabled(false);
    ui->pushButton_remove_topic->setEnabled(false);
    ui->pushButton_rename_topic->setEnabled(false);
    ui->pushButton_rvizReadDisplaySet->setEnabled(false);
    ui->pushButton_rvizSaveDisplaySet->setEnabled(false);
    ui->label_rvizShow->show();
    ui->button_disconnect->setEnabled(false);
    ui->button_connect->setEnabled(true);
    
}

void cyrobot_monitor::MainWindow::on_pushButton_rvizReadDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getOpenFileName(this, "Import RVIZ Display Setting", "/home/" + getUserName() + "/", "YAML(*.yaml);;ALL(*.*)");
    if (!path.isEmpty())
    {
        map_rviz_->ReadDisplaySet(path);
    }
}

void cyrobot_monitor::MainWindow::on_pushButton_rvizSaveDisplaySet_clicked()
{
    if (map_rviz_ == nullptr)
    {
        return;
    }
    QString path = QFileDialog::getSaveFileName(this, "Export  RVIZ Display Setting", "/home/" + getUserName() + "/", "YAML(*.yaml)");
    
    if (!path.isEmpty())
    {
        if (path.section('/', -1, -1).indexOf('.') < 0)
        {
            path = path + ".yaml";
        }
        map_rviz_->OutDisplaySet(path);
    }
}

QString cyrobot_monitor::MainWindow::getUserName()
{
    QString userPath = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    QString userName = userPath.section("/", -1, -1);
    return userName;
}



void cyrobot_monitor::MainWindow::on_roscore_clicked()
{
    system("gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash ;roscore'&");
}

void cyrobot_monitor::MainWindow::on_rviz_map_clicked()
{
    system("gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash; source /home/chun/realsense_ws/devel/setup.bash ;roslaunch map_saver rs_camera.launch '&");
}

void cyrobot_monitor::MainWindow::on_Realsense_clicked()
{
    system("gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash; source /home/chun/realsense_ws/devel/setup.bash ;roslaunch realsense2_camera rs_camera.launch '&");
}
