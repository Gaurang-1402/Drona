'use client'

import Compass from './Compass'
import GaugeChart from 'react-gauge-chart'
import { TrendingQuestions } from './TrendingQuestions'
import TimeComponent from './TimeComponent'
import { useState, useEffect, useRef } from 'react'
import { toast } from 'react-toastify'
import ROSLIB from 'roslib';

export default function Home() {
  const [lastCommands, setLastCommands] = useState<string[]>([])
  const [currentPrompt, setPrompt] = useState('')

  const [imgSrcFront, setImgSrcFront] = useState<string | null>(null);
  const [imgSrcBottom, setImgSrcBottom] = useState<string | null>(null);

  const canvasRef1 = useRef<HTMLCanvasElement | null>(null);
  const canvasRef2 = useRef<HTMLCanvasElement | null>(null);

  const [latestMessageFront, setLatestMessageFront] = useState<any>(null);
  const [latestMessageBottom, setLatestMessageBottom] = useState<any>(null);

  const UNSPLASH_IMG='https://unsplash.com/photos/OXbYiuWe7RY/download?ixid=M3wxMjA3fDB8MXxzZWFyY2h8NHx8ZHJvbmUlMjBpbWFnZXxlbnwwfHx8fDE2OTI1NTkxNjV8MA&force=true&w=400'

  console.log(imgSrcFront, imgSrcBottom)
  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // replace localhost with your ROS2 machine IP if different
    });

    ros.on('connection', () => {
      console.log('Connected to rosbridge WebSocket server.');
    });

    const imageTopicFront = new ROSLIB.Topic({
      ros: ros,
      name: '/drone/front/image_raw',
      messageType: 'sensor_msgs/msg/Image'
    });

    const imageTopicBottom = new ROSLIB.Topic({
      ros: ros,
      name: '/drone/bottom/image_raw',
      messageType: 'sensor_msgs/msg/Image'
    });


    function base64ToArrayBuffer(base64: any) {
      var binaryString = window.atob(base64);
      var binaryLen = binaryString.length;
      var bytes = new Uint8Array(binaryLen);
      for (let i = 0; i < binaryLen; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }
      return bytes;
    }


    imageTopicFront.subscribe((message: any) => {
      // Update the state with the latest message
      setLatestMessageFront(message);

      if (canvasRef1.current) {
        const canvas = canvasRef1.current;
        const ctx = canvas.getContext('2d');
        if (ctx) {
          const imgData = ctx.createImageData(message.width, message.height);
          let j = 0;

          const rawPixelData = base64ToArrayBuffer(message.data);

          for (let i = 0; i < rawPixelData.length; i += 3) {
            imgData.data[j++] = rawPixelData[i];     // Red
            imgData.data[j++] = rawPixelData[i + 1]; // Green
            imgData.data[j++] = rawPixelData[i + 2]; // Blue
            imgData.data[j++] = 255;                 // Alpha (fully opaque)
          }

          ctx.putImageData(imgData, 0, 0);
          // Convert canvas to image source
          const imageUrl = canvas.toDataURL()

          setImgSrcFront(imageUrl);
        }
      }
    });


    imageTopicBottom.subscribe((message: any) => {
      // Update the state with the latest message
      setLatestMessageBottom(message);


      if (canvasRef2.current) {
        const canvas = canvasRef2.current;
        const ctx = canvas.getContext('2d');
        if (ctx) {
          const imgData = ctx.createImageData(message.width, message.height);
          let j = 0;



          const rawPixelData = base64ToArrayBuffer(message.data);

          for (let i = 0; i < rawPixelData.length; i += 3) {
            imgData.data[j++] = rawPixelData[i];     // Red
            imgData.data[j++] = rawPixelData[i + 1]; // Green
            imgData.data[j++] = rawPixelData[i + 2]; // Blue
            imgData.data[j++] = 255;                 // Alpha (fully opaque)
          }

          ctx.putImageData(imgData, 0, 0);
          // Convert canvas to image source
          const imageUrl = canvas.toDataURL()

          setImgSrcBottom(imageUrl);
        }
      }
    });

  }, []);


  const handleSubmit = async (e: any) => {
    e.preventDefault()
    console.log(currentPrompt)

    // Send the voice command to the robot for further processing
    try {
      toast.info("sending cmd to robot..");
      setPrompt('')

      const data = new URLSearchParams({
        text_command: [...lastCommands, currentPrompt].join('. ')
      })
      // data.append('text_command', [...lastCommands, currentPrompt].join('. '))

      let response = await fetch(`${process.env.NEXT_PUBLIC_SERVER_URL}/rosgpt`, {
        method: "POST",
        headers: {
          "Content-Type": "application/x-www-form-urlencoded"
        },
        body: data,
      });

      let jsonResponse = await response.json();
      console.log("Received response from robot:", jsonResponse);

      if (response.status !== 200) {
        throw new Error(jsonResponse.message ?? 'Something went wrong.')
      }

      if (jsonResponse.error) {
        toast.error(jsonResponse.error)
      } else if (jsonResponse.require_more_info) {
        toast.error(jsonResponse.help_text ?? 'we need more info.')
        setLastCommands([...lastCommands, currentPrompt])
      } else {
        // enjoy!!!!
        setLastCommands([])
        toast.success(jsonResponse.response)
      }
    } catch (error: any) {
      console.error("Error sending command to robot:", error);
      toast.error(error.message)

    }
  }
  return (
    <main className='flex gap-6 min-h-screen'>
      <div className='drona-sidebar fixed w-24 flex flex-col h-screen px-4 py-10'>


      <div className='mb-10 font-bold text-lg flex flex-col items-center'>
        <img src="/drona_logo_small.png" alt="Drona Logo" className="w-12 h-12" />
        <span>Drona</span>
      </div>

        <div className='flex flex-col h-full gap-3'>



          <div className='cursor-pointer'>
            <img className='w-8  mx-auto my-2' src='/first.png' />
          </div>

          <div className='cursor-pointer'>
            <img className='w-8  mx-auto my-2' src='/second.png' />
          </div>

          <div className='cursor-pointer'>
            <img className='w-8  mx-auto my-2' src='/third.png' />
          </div>

          <div className="mt-auto mb-0" />
          <Underline />

          <div className='cursor-pointer'>
            <img className='w-8  mx-auto my-2' src='/fourth.png' />
          </div>

          <div className='cursor-pointer'>
            <img className='w-8  mx-auto my-2' src='/fifth.png' />
          </div>

          <Underline />

          <div className='cursor-pointer'>
            <img className='w-8  mx-auto my-2' src='/sixth.png' />
          </div>

        </div>
      </div>


      <div className="flex ml-24 flex-col">
        <div className='my-10 grid h-fit grid-cols-4 gap-4 mx-5'>
          <div className='col-span-3 grid grid-cols-2 gap-3'>
            <div className='drona-card overflow-hidden rounded-xl max-h-[500px]'>
              <canvas ref={canvasRef1} width={latestMessageFront?.width} height={latestMessageFront?.height} style={{ display: 'none' }}></canvas>
              <img src={imgSrcFront??UNSPLASH_IMG} className='w-full h-full object-fill' alt="Drone Front Camera Feed" />
            </div>
            <div className='drona-card overflow-hidden rounded-xl'>
              <canvas ref={canvasRef2} width={latestMessageBottom?.width} height={latestMessageBottom?.height} style={{ display: 'none' }}></canvas>
              <img src={imgSrcBottom??UNSPLASH_IMG} className='w-full h-full object-fill' alt="Drone Front Camera Feed" />
            </div>
          </div>


          <div className='drona-card px-7 py-5 text-sm h-fit rounded-xl'>
            <div className='flex justify-between'>
              <div>
                <div className='font-bold text-lg'>
                  Drona Model Z
                </div>
                <div className='text-gray-500 text-sm' >
                  AI powered FHD drone with 4K camera
                </div>
              </div>
              <div>
                <img className='w-12' src='/icons8-drone-90.png' />
              </div>
            </div>

            <Underline />


            <div className='flex justify-between'>
              <div className='text-pink-400 text-base text-sm font-bold'>
                <TimeComponent />
              </div>

              <div className='text-green-400 font-bold'>
                Online
              </div>
            </div>

            <Underline />

            <div className='flex justify-between'>
              <div>
                <div className=''>
                  Battery status
                </div>
                <div className='text-gray-500'>
                  4 min ago
                </div>
              </div>
              <div className='text-yellow-400 font-bold'>
                78%
              </div>
            </div>
            <Underline />
          </div>




          <div className='drona-card px-7 py-5  rounded-xl col-span-3'>

            <div className="flex gap-2">
              <form className='w-full' onSubmit={handleSubmit}>
                <input required value={currentPrompt} onChange={(e) => setPrompt(e.target.value)} className='w-full rounded px-3 py-3 text-md text-[#222224]' placeholder='instruct your drone to do something cool. 😎' />
              </form>

              <img className='w-12 ml-2' src="/mic_icon.png" alt="microphone--v1" />
            </div>

            <TrendingQuestions setPrompt={setPrompt} />

          </div>

          <div className='drona-card col-span-1 px-7 py-5 rounded-xl flex flex-col justify-center items-center'>
            <div className='w-4/5 h-4/5 mb-4'>
              <Compass />
            </div>
            <GaugeChart
              id="gauge-chart5"
              nrOfLevels={420}
              arcsLength={[0.3, 0.5, 0.2]}
              colors={['#5BE12C', '#F5CD19', '#EA4228']}
              percent={0.37}
              arcPadding={0.02}
            />
            <span> Speed </span>
          </div>






        </div>

      </div>
    </main>
  )
}


export const Underline = () => (
  <div className='mt-3 mb-4 mx-auto w-11/12 border-b-2 border-gray-700'></div>
)
