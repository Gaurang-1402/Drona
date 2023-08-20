'use client'


export const TrendingQuestions = ({setPrompt}: {setPrompt: any}) => {

    const trendingQuestions = [
        {
            // English:
            language: 'English',
            prompts: [
                "Take off and move forward for 1 meter at a speed of 0.5 meters per second.",
                "Land.",
                // "Move forward for 1 meter at a speed of 0.5 meters per second.",
                // "Move backward for 2 meters at a speed of 0.7 meters per second.",
                // "Move left for 3 meters at a speed of 0.6 meters per second.",
                // "Move right for 4 meters at a speed of 0.5 meters per second.",
                // "Move up for 1 meter at a speed of 0.3 meters per second.",
                // "Move down for 2 meters at a speed of 0.4 meters per second.",

            ]
        },
        {
            language: 'German',
            prompts: [
                "Bewegen Sie sich vorwärts für 1 Meter mit einer Geschwindigkeit von 0,5 Metern pro Sekunde.",

            ]
        },
        {
            language: 'Japanese',
            prompts: [
                "Mae ni 1 meter, 0.5 meters per second no sokudo de susumu.",

            ]
        },
        {
            language: 'Hindi',
            prompts: [
                "Aage badho 1 meter, 0.5 meters per second ke liye.",
            ]
        }
    ]



    return (
        <div>


            <div className="font-bold text-gray-200 mt-5">
                Sample Prompts
            </div>


            <div className="grid grid-cols-3 gap-x-5">
                {trendingQuestions.map((block, indxx) => (
                    <div key={indxx} className="text-sm">
                        <div className="font-bold mt-2 mb-1 text-gray-300">
                            {block.language}
                        </div>
                        <div className="flex gap-2 flex-wrap mb-4">
                            {block.prompts.map((question, indx) => (
                                <div key={indx}>
                                    <div onClick={() => {
                                        setPrompt(question)
                                    }} className="cursor-pointer text-gray-400 px-2 bg-gray-800 border-gray-600 py-1 rounded-lg border w-fit text-xs flex items-center justify-between">
                                        {question}
                                    </div>
                                </div>

                            ))}
                        </div>


                    </div>
                ))}
            </div>
        </div>


    )
}
