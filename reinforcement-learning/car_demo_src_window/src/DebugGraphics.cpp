/*
*	DebugGraphics.cpp
*
*   This is Rocketman's code for point, line and triangle drawing in
*   OGRE. See DebugGraphics.h for full details.
*   
*/

#include <Ogre.h>
#include "DebugGraphics.h"
#include <vector>

namespace Ogre {

// Private Implementation of data for DebugGraphics
class DebugGraphics::PImpl {
public:
    unsigned long currentColour;
    int maxVertices, nVertices;
    HardwareVertexBufferSharedPtr vbuf, cbuf;
    class DebugRenderable *points, *lines, *triangles;
    SceneNode *node;
};

/*
*   class DebugRenderable
*/
class DebugRenderable : public SimpleRenderable {
public:
    std::vector<Vector3> vertices;
    std::vector<unsigned long> colours;
    bool isUsed;
    DebugGraphics::PImpl *p;

    /*
    *   DebugRenderable()
    */
    DebugRenderable(RenderOperation::OperationType type, DebugGraphics::PImpl *_p) : p(_p) {
        mRenderOp.indexData = 0;
        mRenderOp.operationType = type;
        mRenderOp.useIndexes = false;

        VertexData *vdata = mRenderOp.vertexData = new VertexData();
        vdata->vertexDeclaration->addElement(0, 0, VET_FLOAT3, VES_POSITION, 0);
        vdata->vertexDeclaration->addElement(1, 0, VET_COLOUR, VES_DIFFUSE, 0);
        vdata->vertexBufferBinding->setBinding(0, p->vbuf);
        vdata->vertexBufferBinding->setBinding(1, p->cbuf);

        mBox.setExtents(Vector3(-1000,-1000,-1000), Vector3(1000,1000,1000));

        setMaterial("Material/debugGraphics");
    }

    /*
    *   ~DebugRenderable()
    */
    ~DebugRenderable() {
        delete mRenderOp.vertexData;
    }

    
    /*
    *   DebugRenderable::addVertex()
    */
    void addVertex(const Vector3 &v) {
        if (p->nVertices < p->maxVertices) {
            vertices.push_back(v);
            colours.push_back(p->currentColour);
            isUsed = true;
            ++p->nVertices;
        }
    }

    /*
    *   DebugRenderable::everyFrame()
    */
    void everyFrame(Vector3 *vptr, unsigned long *cptr, int &count) {
        setVisible(isUsed);
        if (isUsed) {
            int size = vertices.size();

            if (count+size <= p->maxVertices) {
                mRenderOp.vertexData->vertexStart = count;
                mRenderOp.vertexData->vertexCount = size;
                if (size > 0) {
                    memcpy(vptr+count, &*vertices.begin(), size * sizeof(Vector3));
                    memcpy(cptr+count, &*colours.begin(), size * sizeof(unsigned int));
                }
                count += size;
            }
            else {
                throw; // should be imposible
            }

            /*
            vertices.clear();
            colours.clear();
            isUsed = false;
            */
        }
    }

    void clear()
    {
        vertices.clear();
        colours.clear();
        isUsed = false;
    }

    // some stuff from SimpleRenderable
    Real getBoundingRadius() const { return 1000; }
    Real getSquaredViewDepth(const Camera *cam) const { return 0; }
};

/*
*   DebugGraphics()
*/
DebugGraphics::DebugGraphics(SceneManager *sceneManager, int maxVertices) {
    
    // create material
    Ogre::MaterialPtr myMaterial = Ogre::MaterialManager::getSingleton().create("Material/debugGraphics","General"); 
    myMaterial->setReceiveShadows(false); 
    myMaterial->getTechnique(0)->setLightingEnabled(false);
    myMaterial->getTechnique(0)->getPass(0)->setDepthBias(16);
    myMaterial->getTechnique(0)->getPass(0)->setDepthFunction(Ogre::CMPF_LESS_EQUAL);
    myMaterial->getTechnique(0)->getPass(0)->setPointSize(10);
    
    p = new PImpl();
    p->currentColour = 0xFFFFFFFF;
    p->maxVertices = maxVertices;
    p->nVertices = 0;
    
    // create hardware buffers
    p->vbuf = HardwareBufferManager::getSingleton().createVertexBuffer(
        sizeof(Vector3),
        maxVertices,
        HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY,
        false
    );
    p->cbuf = HardwareBufferManager::getSingleton().createVertexBuffer(
        sizeof(unsigned long),
        maxVertices,
        HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY,
        false
    );
    // create renderables and add to scene
    p->node = sceneManager->getRootSceneNode()->createChildSceneNode();
    p->points    = new DebugRenderable(RenderOperation::OT_POINT_LIST, p);
    p->lines     = new DebugRenderable(RenderOperation::OT_LINE_LIST, p);
    p->triangles = new DebugRenderable(RenderOperation::OT_TRIANGLE_LIST, p);
    p->node->attachObject(p->points);
    p->node->attachObject(p->lines);
    p->node->attachObject(p->triangles);
    
}

/*
*   ~DebugGraphics()
*/
DebugGraphics::~DebugGraphics() {
    // remove and delete everything
    p->node->detachAllObjects();
    SceneNode *parent = (SceneNode*) p->node->getParent();
    parent->removeAndDestroyChild(p->node->getName());

    delete p->points;
    delete p->lines;
    delete p->triangles;
    delete p;
}

/*
*   DebugGraphics::setColour()
*/
void DebugGraphics::setColour(const ColourValue &c) {
    Root::getSingleton().getRenderSystem()->convertColourValue(c, (Ogre::uint32*)&p->currentColour);
}

/*
*   DebugGraphics::addPoint()
*/
void DebugGraphics::addPoint(const Vector3 &a) {
    p->points->addVertex(a);
}

/*
*   DebugGraphics::addLine()
*/
void DebugGraphics::addLine(const Vector3 &a, const Vector3 &b) {
    p->lines->addVertex(a);
    p->lines->addVertex(b);
}

/*
*   DebugGraphics::addTriangle()
*/
void DebugGraphics::addTriangle(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
    p->triangles->addVertex(a);
    p->triangles->addVertex(b);
    p->triangles->addVertex(c);
}

/*
*   DebugGraphics::frameStarted()
*/
bool DebugGraphics::frameStarted(const FrameEvent &evt) {
    int count = 0;
    
    Vector3 *vptr = (Vector3*) p->vbuf->lock(HardwareBuffer::HBL_DISCARD);
    unsigned long *cptr = (unsigned long*) p->cbuf->lock(HardwareBuffer::HBL_DISCARD);

    p->points   ->everyFrame(vptr, cptr, count);
    p->lines    ->everyFrame(vptr, cptr, count);
    p->triangles->everyFrame(vptr, cptr, count);

    p->vbuf->unlock();
    p->cbuf->unlock();

    /*p->nVertices = 0;*/

    return true;
}

void DebugGraphics::clear()
{
    p->points->clear();
    p->lines->clear();
    p->triangles->clear();
    p->nVertices = 0;
}

}
