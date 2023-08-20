'use client'

import Compass from './Compass'
import { TrendingQuestions } from './TrendingQuestions'
import TimeComponent from './TimeComponent'
import { useState } from 'react'
import { toast } from 'react-toastify'

export default function Home() {
  const [lastCommands, setLastCommands] = useState<string[]>([])
  const [currentPrompt, setPrompt] = useState('')


  const handleSubmit = async (e: any) => {
    e.preventDefault()
    console.log(currentPrompt)

    // Send the voice command to the robot for further processing
    try {
      toast.info("sending cmd to robot..");
      setPrompt('')

      let response = await fetch(`${process.env.NEXT_PUBLIC_SERVER_URL}/rosgpt`, {
        method: "POST",
        headers: {
          "Content-Type": "application/x-www-form-urlencoded"
        },
        body: JSON.stringify({
          text_command: [...lastCommands, currentPrompt]
        }),
      });

      let jsonResponse = await response.json();
      console.log("Received response from robot:", jsonResponse);

      if(response.status !== 200){
        throw new Error(jsonResponse.message??'Something went wrong.')
      }

      if(jsonResponse.error){
        toast.error(jsonResponse.error)
      }else if(jsonResponse.require_more_info){
        toast.error(jsonResponse.help_text??'we need more info.')
        setLastCommands([...lastCommands, currentPrompt])
      }else{
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
      <div className='drona-sidebar flex flex-col h-screen px-4 py-10'>


        <div className='mb-10 font-bold text-lg'>
          Drona
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


      <div className="flex flex-col">
        <div className='my-10 grid h-fit grid-cols-4 gap-4 mx-5'>
          <div className='drona-card col-span-3 overflow-hidden rounded-xl'>
            <video loop muted autoPlay={true} className='w-full h-[500px] object-fill overflow-hidden rounded-xl' >
              <source src="/sample_feed.mp4" type="video/mp4" />
            </video>
          </div>


          <div className='drona-card px-7 py-5 text-sm h-fit rounded-xl'>
            <div className='flex justify-between'>
              <div>
                <div className='font-bold'>
                  Drona Model Z
                </div>
                <div className='text-gray-500'>
                  AI powered FHD drone with 4K camera
                </div>
              </div>
              <div>
                <img className='w-12' src='/icons8-drone-90.png' />
              </div>
            </div>

            <Underline />


            <div className='flex justify-between'>
              <div className='text-pink-400 text-base font-bold'>
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

              <img className='w-14' src="https://img.icons8.com/ios-filled/100/ffffff/microphone--v1.png" alt="microphone--v1" />
            </div>

            <TrendingQuestions setPrompt={setPrompt} />

          </div>

          <div className='drona-card col-span-1 px-7 py-5 rounded-xl'>
            <Compass />
          </div>





        </div>

      </div>
    </main>
  )
}


export const Underline = () => (
  <div className='mt-3 mb-4 mx-auto w-11/12 border-b-2 border-gray-700'></div>
)
