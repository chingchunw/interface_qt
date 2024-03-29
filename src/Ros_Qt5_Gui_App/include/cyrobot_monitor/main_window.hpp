/**
 * @file /include/cyrobot_monitor/main_window.hpp
 *
 * @brief Qt based gui for cyrobot_monitor.
 *
 * @date November 2010
 **/
#ifndef cyrobot_monitor_MAIN_WINDOW_H
#define cyrobot_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "addtopics.h"
#include "settings.h"
#include "qrviz.hpp"
//仪表盘头文件
#include "QProcess"
#include <QStandardItemModel>
#include <QTreeWidgetItem>
#include <QSoundEffect>
#include <QComboBox>
#include <QSpinBox>
#include <QVariant>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QInputDialog>
#include <QFileDialog>
#include <map>
//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include<rviz/tool.h>

namespace Ui {
class MainWindow;
}

/*****************************************************************************
** Namespace
*****************************************************************************/
namespace cyrobot_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT
    
    public:
      MainWindow(int argc, char** argv, QWidget *parent = nullptr);
      ~MainWindow();
    
      void ReadSettings(); // Load up qt program settings at startup
      void WriteSettings(); // Save qt program settings when closing
    
      void closeEvent(QCloseEvent *event); // Overloaded function
      void showNoMasterMessage();
        void initRviz();
        void initUis();
        void initTopicList();
    public slots:
      /******************************************
      ** Auto-connections (connectSlotsByName())
      *******************************************/
        void on_actionAbout_triggered();
        void slot_rosShutdown();
        void refreashTopicList();
        void RvizGetModel(QAbstractItemModel *model);
        /******************************************
        ** Manual connections
        *******************************************/
        void updateLoggingView(); // no idea why this can't connect automatically
        void slot_choose_topic(QTreeWidgetItem *choose, QString name);
        void slot_set_2D_Goal();
        void slot_set_2D_Pos();
        void slot_set_select();
        void slot_move_camera_btn();
        //设置界面
        void slot_setting_frame();
    private slots:
       
        void on_pushButton_add_topic_clicked();
        void on_pushButton_remove_topic_clicked();       
        void on_pushButton_rename_topic_clicked();       
        void on_treeView_rvizDisplayTree_clicked(const QModelIndex &index);      
        void on_button_disconnect_clicked();
        
        /// \brief 导入rviz Display配置
        void on_pushButton_rvizReadDisplaySet_clicked();
        /// \brief 导出rviz Display配置
        void on_pushButton_rvizSaveDisplaySet_clicked();
        void on_button_connect_clicked();      
        void on_roscore_clicked();
        void on_rviz_map_clicked();
        void on_Realsense_clicked();

private:
        Ui::MainWindowDesign *ui;
        void initData();
        QString JudgeDisplayNewName(QString name);
      
        void connections();        
        void inform(QString);
        bool AskInform(QString);
        QString getUserName();
        
        QNode qnode;
        QProcess *quick_cmd = nullptr;
        QProcess *close_remote_cmd = nullptr;
        QProcess *base_cmd = nullptr;
        QRviz *map_rviz_ = nullptr;
        QStandardItemModel* treeView_rviz_model = nullptr;
        AddTopics *addtopic_form = nullptr;
        //存放rviz treewidget当前显示的控件及控件的父亲的地址
        QMap <QWidget*,QTreeWidgetItem *> widget_to_parentItem_map;
        //存放状态栏的对应关系 display名 状态item
        QMap <QString,QTreeWidgetItem *> tree_rviz_stues;
        //存放display的当前值 item名，参数名称和值
        QMap <QTreeWidgetItem*,QMap<QString,QString>> tree_rviz_values;
        Settings *set = nullptr;
        
        QAbstractItemModel* m_modelRvizDisplay;
        
        QMap<QString, QString> m_mapRvizDisplays;
        
        QString m_sRvizDisplayChooseName_;

};
}// namespace cyrobot_monitor

#endif // cyrobot_monitor_MAIN_WINDOW_H
