drunken-ironman
===============

#Requirements:
- PCL 1.5
- OpenCV 2.4
- OpenNI latest unstable
- SensorKinect latest stable

    boost::function<void
    (const boost::shared_ptr<openni_wrapper::Image>&)> rgb = boost::bind (&KinectGrabberWriter::image_cb_, this, _1);
    boost::function<void
    (const boost::shared_ptr<openni_wrapper::DepthImage>&)> depth = boost::bind (&KinectGrabberWriter::image_cb_, this, _1);
    interface->registerCallback (f);
    interface->registerCallback (g);

