/*
*	DebugGraphics.h
*
*   This is Rocketman's code for point, line and triangle drawing in
*   OGRE.
*   
*   He has posted it online here:
*   http://www.ogre3d.org/forums/viewtopic.php?p=106095
*
*   There is no formal licence, but based on the above forum post, it
*   seems clear that he intends for people to use the code in their
*   own projects, so I assume he won't mind my using it here.
*
*   The code is also available as part of Nogredex:
*   http://www.ogre3d.org/forums/viewtopic.php?t=5923
*   https://ogreaddons.svn.sourceforge.net/svnroot/ogreaddons/trunk/nogredex/
*
*   Note that I (Stephen Thompson) have made one or two changes to
*   Rocketman's original code.
*   
*/

#ifndef __DEBUGGRAPHICS_H__
#define __DEBUGGRAPHICS_H__

namespace Ogre {

class DebugGraphics : public FrameListener {

public:
    DebugGraphics(Ogre::SceneManager *sceneMgr, int maxVertices = 65536);
    ~DebugGraphics();

    void addPoint(const Vector3 &p);
    void addLine(const Vector3 &a, const Vector3 &b);
    void addTriangle(const Vector3 &a, const Vector3 &b, const Vector3 &c);
    void setColour(const ColourValue &c);

    void clear();

    bool frameStarted(const Ogre::FrameEvent &evt);

    class PImpl;

private:
    PImpl *p;
};

}

#endif
