#include "x264_image_transport/x264_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <cstdio> //for memcpy

namespace x264_image_transport
{

	namespace enc = sensor_msgs::image_encodings;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Constructor.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	x264Publisher::x264Publisher()
		: encFmtCtx_(NULL),
			encCdcCtx_(NULL),
			encFrame_(NULL),
			buffer_(NULL),
			sws_ctx_(NULL),
			initialized_(false),
			qmax_(51)
	{
		pthread_mutex_init(&mutex_,NULL);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Destructor.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	x264Publisher::~x264Publisher()
	{
		pthread_mutex_lock(&mutex_);

		memory_cleanup();

		pthread_mutex_unlock(&mutex_);
		pthread_mutex_destroy(&mutex_);
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  advertiseImpl.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void x264Publisher::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
																		const image_transport::SubscriberStatusCallback  &user_connect_cb,
																		const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
																		const ros::VoidPtr &tracked_object, bool latch)
	{

		//TODO UNDERSTAND THOSE PARAMETERS...


		// queue_size doesn't account for the 3 header packets, so we correct (with a little extra) here.
		queue_size += 4;
		// Latching doesn't make a lot of sense with this transport. Could try to save the last keyframe,
		// but do you then send all following delta frames too?
		latch = false;
		typedef image_transport::SimplePublisherPlugin<x264_image_transport::x264Packet> Base;
		Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

		// Set up reconfigure server for this topic
#ifndef __APPLE__
		reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
		ReconfigureServer::CallbackType f = boost::bind(&x264Publisher::configCb, this, _1, _2);
		reconfigure_server_->setCallback(f);
#endif
	}

#ifndef __APPLE__
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Callback to update the parameters based on dynamic reconfigure.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void x264Publisher::configCb(Config& config, uint32_t level)
	{
		qmax_ = config.qmax;

		// Reinitialize codec
		if (initialized_) {
			pthread_mutex_lock(&mutex_);
			initialized_ = false;
			memory_cleanup();
			pthread_mutex_unlock(&mutex_);
		}
	}
#endif

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Codec memory cleanup.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void x264Publisher::memory_cleanup() const
	{
		// Cleanup memory
		if (encCdcCtx_) {
			avcodec_close(encCdcCtx_);
			av_free(encCdcCtx_);
			encCdcCtx_ = NULL;
		}

		if (encFmtCtx_) {
			avformat_close_input(&encFmtCtx_);
			encFmtCtx_ = NULL;
		}

		if (encFrame_) {
			av_freep(&encFrame_->data[0]);
			av_frame_free(&encFrame_);
			encFrame_ = NULL;
		}

		if (sws_ctx_) {
			sws_freeContext(sws_ctx_);
			sws_ctx_ = NULL;
		}

		if (buffer_) {
			delete [] buffer_;
			buffer_ = NULL;
		}

		initialized_ = false;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Initialize codec.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void x264Publisher::initialize_codec(int width, int height, int fps, const std::string& encoding) const
	{
		pthread_mutex_lock(&mutex_);

		int res = 0;

		//-------------------------------------------------------------------------//
		// Codec log level. Silence all the errors/warnings.
		//-------------------------------------------------------------------------//
		av_log_set_level(AV_LOG_VERBOSE);

		//-------------------------------------------------------------------------//
		// Register codecs, devices and formats.
		//-------------------------------------------------------------------------//
		av_register_all();
		avformat_network_init();

		//-------------------------------------------------------------------------//
		// Codec.
		//-------------------------------------------------------------------------//
		AVCodec *codec = NULL;

		codec = avcodec_find_encoder(AV_CODEC_ID_H264);
		//codec = avcodec_find_encoder_by_name("mjpeg");
		//codec = avcodec_find_encoder_by_name("h264_nvenc");
		if (!codec) {
			ROS_ERROR("x264Publisher :: Codec not found");
			memory_cleanup();
			pthread_mutex_unlock(&mutex_);
			return;
		}

		//-------------------------------------------------------------------------//
		// Context.
		//-------------------------------------------------------------------------//
		encCdcCtx_ = avcodec_alloc_context3(codec);
		if (!encCdcCtx_) {
			ROS_ERROR("x264Publisher :: Cannot allocate encoder context");
			memory_cleanup();
			pthread_mutex_unlock(&mutex_);
			return;
		}

		// Setup some parameter
		/* put sample parameters */
		encCdcCtx_->bit_rate = 512000; // Seems a good starting point
		encCdcCtx_->qmax = qmax_; // Allow big degradation
		/* resolution must be a multiple of two */
		encCdcCtx_->width = width;
		encCdcCtx_->height = height;
		/* frames per second */
		encCdcCtx_->time_base = (AVRational){1,fps};

		// Theory : High gop_size (more b-frame and p-frame) = High CPU, Low Bandwidth
		//        : Low gop_size (more intra-frame)  = Low CPU, High Bandwidth
		encCdcCtx_->gop_size = (1/av_q2d(encCdcCtx_->time_base))/2; // Emit one group of picture (which has an intra frame) every frameRate/2
		encCdcCtx_->max_b_frames = 0;
		encCdcCtx_->pix_fmt = AV_PIX_FMT_YUV420P; //AV_PIX_FMT_YUVJ422P; //AV_PIX_FMT_YUV420P;

		av_opt_set(encCdcCtx_->priv_data, "profile", "main", AV_OPT_SEARCH_CHILDREN);
		av_opt_set(encCdcCtx_->priv_data, "tune", "zerolatency", AV_OPT_SEARCH_CHILDREN);
		av_opt_set(encCdcCtx_->priv_data, "preset", "ultrafast", AV_OPT_SEARCH_CHILDREN);

		// Open the encoder codec
		if (avcodec_open2(encCdcCtx_, codec, NULL) < 0)
		{
			ROS_ERROR("x264Publisher :: Cannot open encoder");
			memory_cleanup();
			pthread_mutex_unlock (&mutex_);
			return;
		}

		//-------------------------------------------------------------------------//
		// Allocate an AVFrame for encoder.
		//-------------------------------------------------------------------------//
		encFrame_ = av_frame_alloc();

		if (!encFrame_) {
			ROS_ERROR("x264Publisher :: Cannot allocate video frame");
			memory_cleanup();
			pthread_mutex_unlock (&mutex_);
			return;
		}

		encFrame_->width = width;
		encFrame_->height = height;

		// Prepare the software scale context
		// Will convert from RGB24 to YUV420P
		if (encoding == enc::BGR8) {
			ROS_INFO("encoding: BGR8");
			sws_ctx_ = sws_getContext(width, height, AV_PIX_FMT_BGR24, //src
																encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
																SWS_FAST_BILINEAR, NULL, NULL, NULL);
			if (!sws_ctx_) {
				ROS_ERROR("x264Publisher :: Cannot create scale context for conversion: %s -> %s",
									av_get_pix_fmt_name(AV_PIX_FMT_BGR24), av_get_pix_fmt_name(encCdcCtx_->pix_fmt));
				memory_cleanup();
				pthread_mutex_unlock (&mutex_);
				return;
			}
			encFrame_->format = AV_PIX_FMT_BGR24;
		}
		else if (encoding == enc::RGB8) {
			ROS_INFO("encoding: RGB8");
			sws_ctx_ = sws_getContext(width, height, AV_PIX_FMT_RGB24, //src
																encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
																SWS_FAST_BILINEAR, NULL, NULL, NULL);
			if (!sws_ctx_) {
				ROS_ERROR("x264Publisher :: Cannot create scale context for conversion: %s -> %s",
									av_get_pix_fmt_name(AV_PIX_FMT_BGR24), av_get_pix_fmt_name(encCdcCtx_->pix_fmt));
				memory_cleanup();
				pthread_mutex_unlock (&mutex_);
				return;
			}
			encFrame_->format = AV_PIX_FMT_RGB24;
		}
		else if (encoding == enc::RGB16) {
			sws_ctx_ = sws_getContext(width, height, AV_PIX_FMT_RGB48, //src
																encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
																SWS_FAST_BILINEAR, NULL, NULL, NULL);
			encFrame_->format = AV_PIX_FMT_RGB48;
		}
		else if (encoding == enc::YUV422) {
			sws_ctx_ = sws_getContext(width, height, AV_PIX_FMT_UYVY422, //src
																encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
																SWS_FAST_BILINEAR, NULL, NULL, NULL);
			encFrame_->format = AV_PIX_FMT_UYVY422;
		}
		else if (encoding == enc::BAYER_GBRG16) {
			/*
			sws_ctx_ = sws_getContext(width, height, PIX_FMT_BAYER_GBRG16LE, //src
																encCdcCtx_->width, encCdcCtx_->height, encCdcCtx_->pix_fmt, //dest
																SWS_FAST_BILINEAR, NULL, NULL, NULL);
			*/
			ROS_WARN_THROTTLE(1.0, "x264Publisher :: Encoding will be supported in next ffmpeg version : %s", encoding.c_str());
			memory_cleanup();
			pthread_mutex_unlock (&mutex_);
			return;
		}
		else {
			ROS_WARN_THROTTLE(1.0, "x264Publisher :: Encoding not supported : %s", encoding.c_str());
			memory_cleanup();
			pthread_mutex_unlock(&mutex_);
			return;
		}

		// Allocate  buffer
		buffer_ = new unsigned char[width * height * 2];

		// Allocate frame
		res = av_image_alloc(encFrame_->data, encFrame_->linesize, width, height, encCdcCtx_->pix_fmt, 32);
		ROS_INFO("width %i, height %i", width, height);
		ROS_INFO("linesizes %i %i %i %i", encFrame_->linesize[0], encFrame_->linesize[1], encFrame_->linesize[2], encFrame_->linesize[3]);
		if (res < 0) {
			ROS_ERROR("x264Publisher :: Cannot allocate image buffer");
			memory_cleanup();
			pthread_mutex_unlock (&mutex_);
			return;
		}

		//-------------------------------------------------------------------------//
		// Initialize packet.
		//-------------------------------------------------------------------------//
		av_init_packet(&encodedPacket_);
		encodedPacket_.data = NULL;    // packet data will be allocated by the encoder
		encodedPacket_.size = 0;

		initialized_ = true;
		ROS_INFO("x264Publisher :: Codec initialized (width: %i, height: %i, fps: %i)", width, height, fps);
		pthread_mutex_unlock(&mutex_);
	}


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Connect callback.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void x264Publisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
	{
		ROS_INFO("x264Publisher :: connectCallback");
		//This will always occur after the first publish...
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Publish callback.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void x264Publisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
	{
		int width = message.width;
		int height = message.height;
		int fps = 24;
		int srcstride = message.step;
		int res = 0;

		//-------------------------------------------------------------------------//
		// Initialize on 1st publish.
		//-------------------------------------------------------------------------//
		if (!initialized_) {
			initialize_codec(width, height, fps, message.encoding);
			// Something went wrong
			if (!initialized_) {
				ROS_WARN("x264Publisher :: Failed to initialise!");
				return;
			}
		}

		pthread_mutex_lock (&mutex_);

		//-------------------------------------------------------------------------//
		// Let's convert the image to something ffmpeg/x264 understands.
		//-------------------------------------------------------------------------//
		//Pointer to RGB DATA
		uint8_t* src_data[1];
		src_data[0] = (uint8_t*)&message.data[0];
		//ROS_INFO("Input data size %i ",message.data.size());

		//ROS_INFO("--------------0");
		res = sws_scale(sws_ctx_, src_data, &srcstride, 0, height, encFrame_->data, encFrame_->linesize);
		//ROS_INFO("--------------1");
		//ROS_INFO("sws_scale result %i", res);
		//ROS_INFO("src_linesize %i, dst_linesize %i", srcstride, encFrame_->linesize[0]);
		//ROS_INFO("src_data[0] %u, dst_data[0] %u", src_data[0][0], encFrame_->data[0][0]);

		//int got_output = 0;
		//uint8_t buffer[height * srcstride]; //one full frame
		//encodedPacket_.data=buffer_;
		//encodedPacket_.size=height * srcstride;

		//-------------------------------------------------------------------------//
		// Encode.
		//-------------------------------------------------------------------------//
		res = avcodec_send_frame(encCdcCtx_, encFrame_);
		//ROS_INFO("--------------2");
		if (res == 0) {
			av_init_packet(&encodedPacket_);
			res = avcodec_receive_packet(encCdcCtx_, &encodedPacket_);
			//ROS_INFO("--------------3");

			//ROS_INFO("OUT:%d, %d",got_output, ret);
			if (res == 0) {
				// OK, Let's send our packets...
				x264_image_transport::x264Packet packet;

				// Set data
				packet.data.resize(encodedPacket_.size);

				// Set width & height
				packet.img_width = width;
				packet.img_height = height;

				// Copy NAL data
				//memcpy(&packet.data[0],buffer_,ret);
				memcpy(&packet.data[0], encodedPacket_.data, encodedPacket_.size);
				// Affect header
				packet.header = message.header;
				packet.codec = 0;
				// Publish
				//ROS_INFO("Publishing x264 packet %d", packet.codec);
				publish_fn(packet);

				// Not yet used...
				av_packet_unref(&encodedPacket_);
			}
		}

		pthread_mutex_unlock (&mutex_);
	}

} // namespace x264_image_transport
