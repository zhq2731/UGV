#ifndef MAPFILEGENERATION_H
#define MAPFILEGENERATION_H

#include <QDialog>

namespace Ui {
class MapFileGeneration;
}

class MapFileGeneration : public QDialog
{
Q_OBJECT

public:
explicit MapFileGeneration(QWidget *parent = nullptr);
~MapFileGeneration();

void setMapOrigin(const double fLat, const double fLon, const float fHeight);

private:
    void setUIState(bool bFlag);
private slots:
    void on_btGetMapOrigin_clicked();

    void on_btSaveMapFile_clicked();

    void on_pushButton_3_clicked();

    void on_btModifyMapFile_clicked();

private:
Ui::MapFileGeneration *ui;
double m_fLat;
float m_fHeight;
double m_fLon;
float m_fMaxX;
float m_fMinX;
float m_fMaxY;
float m_fMinY;
int m_fImageWidth;
int m_fImageHeight;
float m_fResolution;

double m_fLatInit;
float m_fHeightInit;
double m_fLonInit;

};

#endif // MAPFILEGENERATION_H
