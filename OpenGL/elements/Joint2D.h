#ifndef BLOSSOM_POINT_H
#define BLOSSOM_POINT_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include "../elements/circle.h"

#include "../geometry/geometryutils.h"

#include <vector>
#include <vector>

class OGLTWidget;

class Link2D;

class Joint2D: public Polygon2D
{
public:
    Joint2D(OGLTWidget* parent);

    friend class Link2D;

     // Virtual functions
    virtual void mouse_drag(MouseInfo m);
    virtual bool draw(QMatrix4x4 model, QMatrix4x4 projection);
    virtual QRect getBB()const;

    void set_position(QVector2D pos);
    QVector2D get_position(){return m_pos;}

    void add_link(Link2D* link);
    void remove_link(Link2D* link);

    std::vector<Link2D*> get_links(){return m_links;}
    std::vector<Joint2D*> get_children();
    std::vector<Joint2D*> get_parents();


    virtual ~Joint2D();

private:

    // Visual elements
    Circle2D* m_circle;

    // Connection elements
    QVector2D m_pos;
    std::vector<Link2D*> m_links;


};

#endif //
