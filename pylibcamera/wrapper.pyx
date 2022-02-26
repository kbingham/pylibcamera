import mmap
import time
import hashlib

import logging

from cython import NULL, size_t

from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp.memory cimport unique_ptr, shared_ptr
from libcpp cimport bool
from libc.stdint cimport uint32_t, uint64_t
from posix.unistd cimport close, read, off_t


cdef extern from "sys/types.h":
    ctypedef dev_t;

cdef extern from "libcamera/libcamera.h" namespace "libcamera":
    ctypedef enum ControlType:
        ControlTypeNone,
        ControlTypeBool,
        ControlTypeByte,
        ControlTypeInteger32,
        ControlTypeInteger64,
        ControlTypeFloat,
        ControlTypeString,
        ControlTypeRectangle,
        ControlTypeSize,

    ctypedef enum StreamRole:
        Raw,
        StillCapture,
        VideoRecording,
        Viewfinder

    ctypedef enum NoiseReductionModeEnum:
        NoiseReductionModeOff = 0,
        NoiseReductionModeFast = 1,
        NoiseReductionModeHighQuality = 2,
        NoiseReductionModeMinimal = 3,
        NoiseReductionModeZSL = 4,

    ctypedef enum CC_Status "libcamera::CameraConfiguration::Status":
        Valid "libcamera::CameraConfiguration::Valid"
        Adjusted "libcamera::CameraConfiguration::Adjusted"
        Invalid "libcamera::CameraConfiguration::Adjusted"

    cdef cppclass Private:
        pass

    cdef cppclass Size:
        unsigned int width;
        unsigned int height;

    cdef cppclass Stream:
        const StreamConfiguration &configuration();

    cdef cppclass ControlId:
        ControlId(unsigned int id, const string &name, ControlType type);
        unsigned int id();
        const string &name();

    cdef cppclass ControlValue:
        pass

    cdef cppclass ControlList:
        # void set(unsigned int id, const ControlValue &value);
        const ControlValue &get(unsigned int id) const;
        bool contains(unsigned int id) const;
        bool empty();
        size_t size();

    cdef struct StreamConfiguration:
        # PixelFormat pixelFormat;
        Size size;
        unsigned int stride;
        unsigned int frameSize;

        unsigned int bufferCount;

        Stream *stream();
        void setStream(Stream *stream)

        string toString();

    ctypedef vector[StreamRole] StreamRoles;

    # Request status (NB: Name shifted from libcamera)
    ctypedef enum Rq_Status "libcamera::Request::Status":
        RequestPending "libcamera::Request::RequestPending"
        RequestComplete "libcamera::Request::RequestComplete"
        RequestCancelled "libcamera::Request::RequestCancelled"

    # Request reuse flag
    ctypedef enum ReuseFlag:
        Default = 0,
        ReuseBuffers = (1 << 0),


    cdef cppclass Request:
        uint32_t sequence();
        uint64_t cookie();
        Rq_Status status();
        void reuse(ReuseFlag flags = Default);
        ControlList &controls()
        ControlList &metadata()
        # const BufferMap &buffers() const { return bufferMap_; }
        int addBuffer(const Stream *stream, FrameBuffer *buffer)
        # std::unique_ptr<Fence> fence = nullptr);
        FrameBuffer *findBuffer(const Stream *stream);

    cdef cppclass SharedFD:
        # explicit SharedFD(const int &fd = -1);
        # explicit SharedFD(int &&fd);
        # explicit SharedFD(UniqueFD fd);
        # SharedFD(const SharedFD &other);
        # SharedFD(SharedFD &&other);
        # ~SharedFD();
          
        # SharedFD &operator=(const SharedFD &other);
        # SharedFD &operator=(SharedFD &&other);

        bool isValid()
        int get()
        # UniqueFD dup() const;


    ctypedef struct Plane "libcamera::FrameBuffer::Plane":
        unsigned int kInvalidOffset
        SharedFD fd
        unsigned int offset
        unsigned int length

    cdef cppclass FrameBuffer:
        FrameBuffer(const vector[Plane] &planes, unsigned int cookie = 0);
        FrameBuffer(unique_ptr[Private] d, const vector[Plane] &planes, unsigned int cookie = 0);

        const vector[Plane] &planes()
        Request *request() const
        const FrameMetadata &metadata()
        unsigned int cookie()
        void setCookie(unsigned int cookie)
        # unique_ptr[Fence] releaseFence();
        void cancel()

    # In FrameMetadata
    # ctypedef struct Plane:
    #     unsigned int bytesused;

    # In FrameMetadata (NB: Name change)
    ctypedef enum FM_Status "libcamera::FrameMetadata::Status":
        FrameSuccess "libcamera::FrameMetadata::FrameSuccess"
        FrameError "libcamera::FrameMetadata::FrameError"
        FrameCancelled "libcamera::FrameMetadata::FrameCancelled"

    ctypedef struct FrameMetadata:
        FM_Status status;
        unsigned int sequence;
        uint64_t timestamp;

        # Span[Plane] planes()
        # Span[const Plane] planes();

    cdef cppclass FrameBufferAllocator:
        FrameBufferAllocator(shared_ptr[Camera] camera);
        # ~FrameBufferAllocator();

        int allocate(Stream *stream);
        int free(Stream *stream);

        bool allocated();
        const vector[unique_ptr[FrameBuffer]] &buffers(Stream *stream);

    cdef cppclass ReqSignal "libcamera::Signal<Request*>":
        Signal()
        void connect( void (*f_ptr)(Request* req) )
        void disconnect( void (*f_ptr)(Request* req) )

    cdef cppclass Camera:
        int acquire();
        int release();

        int start(const ControlList *controls);
        int stop();
        int configure();
        string id() const;

        # NOTE(meawoppl) - Sketchy: this is defined in libcamera as `using StreamRoles = std::vector<StreamRole>;`
        unique_ptr[CameraConfiguration] generateConfiguration(const vector[StreamRole] &roles);
        int configure(CameraConfiguration *config);
        const ControlList &properties() const;
        unique_ptr[Request] createRequest(uint64_t cookie = 0);
        int queueRequest(Request *request);
        ReqSignal requestCompleted;

    cdef cppclass CameraManager:
        CameraManager();
        # ~CameraManager();

        vector[shared_ptr[Camera]] cameras() const;
        int start();
        void stop();
        Camera get(dev_t devnum);

    cdef cppclass CameraConfiguration:
        int start();
        void stop();
        int size();
        StreamConfiguration &at(unsigned int index);
        CC_Status validate();


cdef class PyCameraManager:
    cdef CameraManager* cm;

    def __cinit__(self):
        # Build/init the camera manager framework
        self.cm = new CameraManager()
        self.cm.start()

        n_cameras = self.cm.cameras().size()
       
        logging.info(f"# Cameras Detected: {n_cameras}")
        cams = self.cm.cameras()
        for c in cams:
            logging.info(f"- {c.get().id().decode()}")

    def get_n_cameras(self):
        return self.cm.cameras().size()

    # def get_camera(self, int index):
    #     return self.cm.cameras()[index]

    def __dealloc__(self):
        self.cm.stop()  
        logging.info("Stopped camera manager")


cdef void request_callback(Request *request):
    logging.warn("Got a callback!!!")


cdef class LibCameraWrapper:
    cdef CameraManager* cm;
    cdef shared_ptr[Camera] camera;
    cdef int debug;
    cdef unique_ptr[CameraConfiguration] camera_cfg;
    cdef StreamConfiguration stream_cfg;
    cdef FrameBufferAllocator* allocator;
    cdef vector[FrameBuffer*]* buffers;
    cdef unique_ptr[Request] request;
    mmaps = {};

    def __cinit__(self, int index):
        # Build/init the camera manager framework
        self.cm = new CameraManager()
        self.cm.start()

        n_cameras = self.cm.cameras().size()
        assert n_cameras > 0, "No cameras detected"
        
        logging.info(f"# Cameras Detected: {n_cameras}")
        cams = self.cm.cameras()
        for c in cams:
            logging.info(f"- {c.get().id().decode()}")

        # Get a specific camera to wrap
        self.camera = self.cm.cameras()[index]
        assert self.camera.get().acquire() == 0, "Failed to acquire camera"

        logging.info(f"Sucessfully acquired camera: {self.camera.get().id().decode()}")

        # Generate a configuration that support raw stills
        # NOTE(meawoppl) - the example I am following uses this, but lib barfs when I add "StillCapture" and "Raw", so IDK
        # self.config = self.camera.generateConfiguration([StreamRole.StillCapture, StreamRole.Raw])   
        self.camera_cfg = self.camera.get().generateConfiguration([StreamRole.StillCapture])   
        assert self.camera_cfg.get() is not NULL

        n_cam_configs = self.camera_cfg.get().size()
        logging.info(f"# Camera Stream Configurations: {n_cam_configs}")
        for i in range(n_cam_configs):
            cfg = self.camera_cfg.get().at(i)
            logging.info(f"Config #{i} - '{cfg.toString().c_str().decode()}'")  

        # TODO(meawoppl) change config settings before camera.configre()
        assert self.camera_cfg.get().validate() == CC_Status.Valid
        assert self.camera.get().configure(self.camera_cfg.get()) >= 0

        logging.info("Using stream config #0")
        self.stream_cfg = self.camera_cfg.get().at(0)
        
        logging.info("Allocating buffers")
        # Allocate buffers for the camera/stream pair
        self.allocator = new FrameBufferAllocator(self.camera)
        assert self.allocator.allocate(self.stream_cfg.stream()) >= 0, "Buffers did not allocate?"
        assert self.allocator.allocated(), "Buffers did not allocate?"

        # The unique_ptr make it so we can't reify this object...
        self.buffers = new vector[FrameBuffer*]()
        n_buffers = self.allocator.buffers(self.stream_cfg.stream()).size()
        logging.info(f"{n_buffers} buffers allocated")
        for buff_num in range(n_buffers):
            self.buffers.push_back(self.allocator.buffers(self.stream_cfg.stream()).at(i).get())
            b = self.allocator.buffers(self.stream_cfg.stream()).at(i).get()

            n_planes = b.planes().size()
            for plane_num in range(n_planes):
                plane_fd = b.planes().at(plane_num).fd.get()
                plane_off = b.planes().at(plane_num).offset
                plane_len = b.planes().at(plane_num).length
                logging.info(f"Buffer #{buff_num} Plane #{plane_num} FD: {plane_fd} Offset: {plane_off} Len: {plane_len}")

                if plane_fd not in self.mmaps:
                    self.mmaps[plane_fd] = mmap.mmap(
                        plane_fd,
                        plane_len,
                        flags=mmap.MAP_SHARED,
                        prot=mmap.PROT_WRITE|mmap.PROT_READ,
                        access=mmap.ACCESS_DEFAULT,
                        offset=plane_off)

        logging.info("Created memory maps:")
        for fd, mp in self.mmaps.items():
            h = hashlib.sha256(mp)
            logging.info(f"FD:{fd} = {mp} hash: {h.hexdigest()}")

        logging.info("Creating requests")
        self.request = self.camera.get().createRequest()
        assert self.request.get() is not NULL, "Failed to create request object?"
        self.request.get().addBuffer(self.stream_cfg.stream(), self.buffers.at(0))
        logging.info("Added buffers")

        logging.info("Starting camera")
        self.camera.get().start(NULL)
        
        # logging.info("Setup callback")
        # self.camera.get().requestCompleted.connect(request_callback)

        logging.info("Queueing request")
        self.camera.get().queueRequest(self.request.get())

        for i in range(100):
            status = self.request.get().status()
            logging.info(f"Request status: {status}")
            if status == RequestComplete:
                break
            time.sleep(0.1)

        logging.info("Created memory maps:")
        for fd, mp in self.mmaps.items():
            h = hashlib.sha256(mp)
            logging.info(f"FD:{fd} = {mp} hash: {h.hexdigest()}")

        self.camera.get().stop()
        logging.info("Stopped camera")

    def __dealloc__(self):
        if self.allocator is not NULL:
            assert self.allocator.free(self.stream_cfg.stream()) >= 0, "Couldn't deallocate buffers?"
            del self.allocator

        if self.camera.get():
            self.camera.get().release()
            logging.info("Released Camera")

        self.cm.stop()  
        logging.info("Stopped camera manager")