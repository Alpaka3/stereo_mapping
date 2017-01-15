using UnityEngine;
using System.Collections;

#if UNITY_5_3 || UNITY_5_3_OR_NEWER
using UnityEngine.SceneManagement;
#endif
using OpenCVForUnity;
using System;

namespace OpenCVForUnitySample
{
    /// <summary>
    /// WebCamTexture to mat sample.
    /// </summary>
    public class StereoMapping : MonoBehaviour
    {

        /// <summary>
        /// The name of the device.
        /// </summary>
        public string requestDeviceName = null;

        /// <summary>
        /// The width.
        /// </summary>
        public int requestWidth = 640;

        /// <summary>
        /// The height.
        /// </summary>
        public int requestHeight = 480;

        /// <summary>
        /// Should use front facing.
        /// </summary>
        public bool requestIsFrontFacing = false;

		/// <summary>
		/// The number of capture
		/// </summary>
		public int captureNum = 0;

        /// <summary>
        /// The web cam texture.
        /// </summary>
        WebCamTexture webCamTexture;

        /// <summary>
        /// The web cam device.
        /// </summary>
        WebCamDevice webCamDevice;

        /// <summary>
        /// The rgba mat.
        /// </summary>
        Mat rgbaMat;

        /// <summary>
        /// The Captured Mat.
        /// </summary>
        /*
                public Mat capMat1 = new Mat ();
                public Mat capMat2 = new Mat ();
        */
        Texture2D LeftTexture;
        Texture2D RightTexture;

        /// <summary>
        /// The colors.
        /// </summary>
        Color32[] colors;

        /// <summary>
        /// The texture.
        /// </summary>
        Texture2D texture;

        /// <summary>
        /// The init waiting.
        /// </summary>
        bool initWaiting = false;

        /// <summary>
        /// The init done.
        /// </summary>
        bool initDone = false;

		/// <summary>
		///  The Capture Done.
		/// </summary>
		bool captureDone = false;

        // Use this for initialization
        void Start ()
        {
            init ();
        }

        /// <summary>
        /// Init of web cam texture.
        /// </summary>
        private void init ()
        {
            if (initWaiting)
                return;

            StartCoroutine (init_coroutine ());
        }

        /// <summary>
        /// Init of web cam texture.
        /// </summary>
        /// <param name="deviceName">Device name.</param>
        /// <param name="requestWidth">Request width.</param>
        /// <param name="requestHeight">Request height.</param>
        /// <param name="requestIsFrontFacing">If set to <c>true</c> request is front facing.</param>
        /// <param name="OnInited">On inited.</param>
        private void init (string deviceName, int requestWidth, int requestHeight, bool requestIsFrontFacing)
        {
            if (initWaiting)
                return;

            this.requestDeviceName = deviceName;
            this.requestWidth = requestWidth;
            this.requestHeight = requestHeight;
            this.requestIsFrontFacing = requestIsFrontFacing;

            StartCoroutine (init_coroutine ());
        }

		private void processBlurring ()
		{
            /*
            Texture2D LeftTexture = Resources.Load("tsukuba_l") as Texture2D;
            Texture2D RightTexture = Resources.Load("tsukuba_r") as Texture2D;
            */
            double resize_ratio = 1;

            Mat grayMat_l0 = new Mat(LeftTexture.height, LeftTexture.width, CvType.CV_8UC1);
            Mat grayMat_r0 = new Mat(RightTexture.height, RightTexture.width, CvType.CV_8UC1);
            Mat grayMat_l = new Mat((int)(grayMat_l0.height() / resize_ratio), (int)(grayMat_l0.width() / resize_ratio), CvType.CV_8UC1);
            Mat grayMat_r = new Mat((int)(grayMat_r0.height() / resize_ratio), (int)(grayMat_r0.width() / resize_ratio), CvType.CV_8UC1);
            Utils.texture2DToMat(LeftTexture, grayMat_l0);
            Utils.texture2DToMat(RightTexture, grayMat_r0);
            Size dsize = new Size(grayMat_l.height(), grayMat_l.width());

            Imgproc.resize(grayMat_l0, grayMat_l, dsize);
            Imgproc.resize(grayMat_r0, grayMat_r, dsize);

            Mat disparity_data = new Mat (grayMat_l.rows(), grayMat_l.cols(), CvType.CV_16S);
            Mat disparity_map = new Mat ();
            Mat imgDisparity16S = new Mat(grayMat_l.rows(), grayMat_l.cols(), CvType.CV_16S);
            Mat imgDisparity8U = new Mat(grayMat_l.rows(), grayMat_l.cols(), CvType.CV_8UC1);

            System.Collections.Generic.List<Mat> mv = new System.Collections.Generic.List<Mat> ();

            /*
            Imgproc.cvtColor(capMat1, grayMat1, Imgproc.COLOR_RGBA2GRAY);
            Imgproc.cvtColor(capMat2, grayMat2, Imgproc.COLOR_RGBA2GRAY);
            */
            double dmax = 0;
            double dmin = 0;

            int m_height = grayMat_l.height();
            int m_width = grayMat_l.width();
            float asp_ratio = ((float)m_height) / ((float)m_width);

            Point[,] maxPointVec = new Point[m_height, m_width];
            double[,] maxValueVec = new double[m_height, m_width];

            Debug.Log("m_height = " + m_height);
            Debug.Log("m_width = " + m_width);
            Core.MinMaxLocResult MatchResult = new Core.MinMaxLocResult();

            DateTime dt1 = DateTime.Now;

            int minDisparity = 0;
            int numDisparities = 16 * 4;
            int blockSize = 21; // SADWindowSize
            int P1 = 0;
            int P2 = 0;
            int disp12MaxDiff = 0;
            int preFilterCap = 0;
            int uniquenessRatio = 0;
            int speckleWindowSize = 100;
            int speckleRange = 100;
            int mode = 0;

            StereoSGBM sbm = StereoSGBM.create(minDisparity, numDisparities, blockSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, mode);

            //StereoBM sbm = StereoBM.create(64, 15);
            sbm.compute(grayMat_l, grayMat_r, imgDisparity16S);

            //normalize to CvType.CV_8U
            Core.normalize(imgDisparity16S, imgDisparity8U, 0, 255, Core.NORM_MINMAX, CvType.CV_8U);

            DateTime dt2 = DateTime.Now;
            Debug.Log((dt2.Second - dt1.Second) + "秒");

            Color32[] matchColor = new Color32[imgDisparity8U.width() * imgDisparity8U.height()];
            short[] short_data = new short[1000];

            OpenCVForUnity.Core.split(imgDisparity16S, mv);
            // comment out to get an element of a matrix.
			Debug.Log ("element1=" + mv[0].get (3, 3, short_data));
            Debug.Log("element2=" + (byte)(short_data[9] * 100));
            Debug.Log("size of disparity mat =" + imgDisparity8U.size());

            Core.MinMaxLocResult minmaxLoc = Core.minMaxLoc(imgDisparity8U);
            dmax = minmaxLoc.maxVal;
            dmin = minmaxLoc.minVal;

            Texture2D CapTexture = new Texture2D(imgDisparity8U.width(), imgDisparity8U.height(), TextureFormat.RGBA32, false);
            Utils.matToTexture2D(imgDisparity8U+grayMat_l/10, CapTexture);
            gameObject.GetComponent<Renderer>().material.mainTexture = CapTexture;

            /*
            Core.fillConvexPoly(Mat     img,
MatOfPoint  points,
Scalar  color,
int     lineType,
int     shift
)*/


            /*
			for c_row=1:r_size-w_size
			for c_col=1:asp_ratio*(r_size-w_size)
					S1=O1(c_row:c_row+w_size-1,c_col:c_col+w_size*asp_ratio-1);
			[max_vec,row_vec] = max(normal_corr(S1,O2));

			[max_val,max_col(c_row,c_col)] = max(max_vec);
			max_row(c_row,c_col) = row_vec(max_col(c_row,c_col));
			endfor
			endfor

			for row=1:size(max_row,1)
					for col=1:size(max_row,2)
						depth_map(row,col,1)=max_row(row,col)-row; %x
					depth_map(row,col,2)=max_col(row,col)-col; %y
			endfor
			endfor

			depth_amp=sqrt(depth_map(:,:,1).^2+depth_map(:,:,2).^2);

			Parallax = zeros(size(O1,1),size(O1,2));
			PFlag = zeros(size(O1,1),size(O1,2));

			for r=1:r_size-w_size
					for c=1:asp_ratio*(r_size-w_size)
						Parallax(r:w_size+r-1, c:w_size*asp_ratio+c-1)=...
						(Parallax(r:w_size+r-1, c:w_size*asp_ratio+c-1).*...
							PFlag(r:w_size+r-1,c:w_size*asp_ratio+c-1)+...
							depth_amp(r,c))./(PFlag(r:w_size+r-1,c:w_size*asp_ratio+c-1)+1);
					PFlag(r:w_size+r-1,c:w_size*asp_ratio+c-1)++;
			endfor
			endfor

			O1=O1/max(max(O1));
			Parallax=Parallax/max(max(Parallax));
			LParallax=Parallax.^2;

			Smap=zeros(size(O1,1),size(O1,2),3);
			Smap(:,:,1)=O1;
			Smap(:,:,2)=O1;
			Smap(:,:,3)=O1;
			Smap(:,:,1)+=LParallax;

			augmentedLParallax=imresize(LParallax,1/Comp_Ratio);

			t1=time
				[X,Y]=meshgrid(-19:20);
			Bmap=zeros(size(OrigImage1,1)+size(X,2),size(OrigImage2,2)+size(Y,1),3);
			for r=1:size(augmentedLParallax,1)
					for c=1:size(augmentedLParallax,2)
						for rgb=1:3
							volatility = 1/(10*(0.01+augmentedLParallax(r,c)));
					Bmap(r:r+size(X,2)-1,c:c+size(Y,1)-1,rgb)+=...
					OrigImage1(r,c,rgb)*1/sqrt(2*pi*volatility)*exp(-(X.^2+Y.^2)/volatility);
			endfor
			endfor
			endfor

			figure
			mesh(Parallax)
			figure
			mesh(depth_amp)
			figure
			imshow(Smap)
			t2=time

				t2-t1
*/


        }

		private void processCapture ()
		{
			Texture2D CapTexture = new Texture2D (webCamTexture.width, webCamTexture.height, TextureFormat.RGBA32, false);
			Utils.webCamTextureToMat (webCamTexture, rgbaMat, colors);
			Utils.matToTexture2D (rgbaMat, CapTexture, colors);
			switch (captureNum) {
			case 0:
				LeftTexture = CapTexture;
				GameObject.Find("Wipe1").GetComponent<Renderer>().material.mainTexture = CapTexture;
				captureNum++;
				break;
			case 1:
                RightTexture = CapTexture;
				GameObject.Find("Wipe2").GetComponent<Renderer>().material.mainTexture = CapTexture;
				captureNum++;
				captureDone = true;
				break;
			case 2:
                LeftTexture = CapTexture;
				GameObject.Find("Wipe1").GetComponent<Renderer>().material.mainTexture = CapTexture;
				captureNum = 1;
				break;
			}
		}

        /// <summary>
        /// Init of web cam texture by coroutine.
        /// </summary>
        private IEnumerator init_coroutine ()
        {
            if (initDone)
                dispose ();

            initWaiting = true;

            if (!String.IsNullOrEmpty (requestDeviceName)) {
                //Debug.Log ("deviceName is "+requestDeviceName);
                webCamTexture = new WebCamTexture (requestDeviceName, requestWidth, requestHeight);
            } else {
                //Debug.Log ("deviceName is null");
                // Checks how many and which cameras are available on the device
                for (int cameraIndex = 0; cameraIndex < WebCamTexture.devices.Length; cameraIndex++) {
                    if (WebCamTexture.devices [cameraIndex].isFrontFacing == requestIsFrontFacing) {

                        //Debug.Log (cameraIndex + " name " + WebCamTexture.devices [cameraIndex].name + " isFrontFacing " + WebCamTexture.devices [cameraIndex].isFrontFacing);
                        webCamDevice = WebCamTexture.devices [cameraIndex];
                        webCamTexture = new WebCamTexture (webCamDevice.name, requestWidth, requestHeight);

                        break;
                    }
                }
            }

            if (webCamTexture == null) {
                if (WebCamTexture.devices.Length > 0) {
                    webCamDevice = WebCamTexture.devices [0];
                    webCamTexture = new WebCamTexture (webCamDevice.name, requestWidth, requestHeight);
                } else {
                    webCamTexture = new WebCamTexture (requestWidth, requestHeight);
                }
            }

            // Starts the camera.
            webCamTexture.Play ();

            while (true) {
                // If you want to use webcamTexture.width and webcamTexture.height on iOS, you have to wait until webcamTexture.didUpdateThisFrame == 1, otherwise these two values will be equal to 16. (http://forum.unity3d.com/threads/webcamtexture-and-error-0x0502.123922/).
#if UNITY_IOS && !UNITY_EDITOR && (UNITY_4_6_3 || UNITY_4_6_4 || UNITY_5_0_0 || UNITY_5_0_1)
                if (webCamTexture.width > 16 && webCamTexture.height > 16) {
#else
                if (webCamTexture.didUpdateThisFrame) {
#if UNITY_IOS && !UNITY_EDITOR && UNITY_5_2
                    while (webCamTexture.width <= 16) {
                        webCamTexture.GetPixels32 ();
                        yield return new WaitForEndOfFrame ();
                    } 
#endif
#endif

                    Debug.Log ("name " + webCamTexture.name + " width " + webCamTexture.width + " height " + webCamTexture.height + " fps " + webCamTexture.requestedFPS);
                    Debug.Log ("videoRotationAngle " + webCamTexture.videoRotationAngle + " videoVerticallyMirrored " + webCamTexture.videoVerticallyMirrored + " isFrongFacing " + webCamDevice.isFrontFacing);

                    initWaiting = false;
                    initDone = true;

                    onInited ();

                    break;
                } else {
                    yield return 0;
                }
            }
        }

        /// <summary>
        /// Releases all resource.
        /// </summary>
        private void dispose ()
        {
            initWaiting = false;
            initDone = false;

            if (webCamTexture != null) {
                webCamTexture.Stop ();
                webCamTexture = null;
            }
            if (rgbaMat != null) {
                rgbaMat.Dispose ();
                rgbaMat = null;
            }
        }

        /// <summary>
        /// Init completion handler of the web camera texture.
        /// </summary>
        private void onInited ()
        {
            if (colors == null || colors.Length != webCamTexture.width * webCamTexture.height)
                colors = new Color32[webCamTexture.width * webCamTexture.height];
            if (texture == null || texture.width != webCamTexture.width || texture.height != webCamTexture.height)
                texture = new Texture2D (webCamTexture.width, webCamTexture.height, TextureFormat.RGBA32, false);

            rgbaMat = new Mat (webCamTexture.height, webCamTexture.width, CvType.CV_8UC4);

            gameObject.GetComponent<Renderer> ().material.mainTexture = texture;

            gameObject.transform.localScale = new Vector3 (webCamTexture.width, webCamTexture.height, 1);
            Debug.Log ("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " + Screen.orientation);



            float width = rgbaMat.width ();
            float height = rgbaMat.height ();

            float widthScale = (float)Screen.width / width;
            float heightScale = (float)Screen.height / height;
            if (widthScale < heightScale) {
                Camera.main.orthographicSize = (width * (float)Screen.height / (float)Screen.width) / 2;
            } else {
                Camera.main.orthographicSize = height / 2;
            }

        }

        // Update is called once per frame
        void Update ()
        {
            if (initDone && webCamTexture.isPlaying && webCamTexture.didUpdateThisFrame) {
                Utils.webCamTextureToMat (webCamTexture, rgbaMat, colors);

                Imgproc.putText (rgbaMat, "W:" + rgbaMat.width () + " H:" + rgbaMat.height () + " SO:" + Screen.orientation, new Point (5, rgbaMat.rows () - 10), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Imgproc.LINE_AA, false);

                Utils.matToTexture2D (rgbaMat, texture, colors);
            }
        }

        /// <summary>
        /// Raises the disable event.
        /// </summary>
        void OnDisable ()
        {
            dispose ();
        }

        /// <summary>
        /// Raises the back button event.
        /// </summary>
        public void OnBackButton ()
        {
#if UNITY_5_3 || UNITY_5_3_OR_NEWER
            SceneManager.LoadScene ("OpenCVForUnitySample");
#else
            Application.LoadLevel ("OpenCVForUnitySample");
#endif
        }

        /// <summary>
        /// Raises the play button event.
        /// </summary>
        public void OnPlayButton ()
        {
            if (initDone)
                webCamTexture.Play ();
        }

        /// <summary>
        /// Raises the pause button event.
        /// </summary>
        public void OnPauseButton ()
        {
            if (initDone)
                webCamTexture.Pause ();
        }

        /// <summary>
        /// Raises the stop button event.
        /// </summary>
        public void OnCaptureButton ()
        {
            if (initDone)
                processCapture ();
        }

        /// <summary>
        /// Raises the change camera button event.
        /// </summary>
        public void OnProcessButton ()
        {
            if (initDone && captureDone) {
				webCamTexture.Pause ();
				processBlurring ();
				webCamTexture.Play ();

			} else {
				Debug.Log ("ERROR! init or capture have not be done. ");
			}
        }
    }
}

