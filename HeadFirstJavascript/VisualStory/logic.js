let curScene = 0
let backScene = 0
let backDecision = null
const decisionStack = []

const setVisibility = (elementId, visible) => {
    element = document.getElementById(elementId)
    visible ? element.style.display = 'inline-block' :  element.style.display = 'none'
}

const setSceneImg = (id) => {
    document.getElementById('scenes').src = `./scene${id}.png`
}

const setControls = curScene => {
    switch (curScene) {
        case 0:
            setVisibility('decision1', true)    
            setVisibility('decision2', false)
            setVisibility('reset', false)
            setVisibility('back', false)
            break;
        case 1:
            setVisibility('decision1', true)    
            setVisibility('decision2', true)
            setVisibility('reset', true)
            setVisibility('back', true)
            break;
        case 2:
            setVisibility('decision1', true)    
            setVisibility('decision2', true)
            setVisibility('reset', true)
            setVisibility('back', true)
            break;
        case 3:
            setVisibility('decision1', true)    
            setVisibility('decision2', true)
            setVisibility('reset', true)
            setVisibility('back', true)
            break;
        case 4:
            setVisibility('decision1', true)
            setVisibility('decision2', true)
            setVisibility('reset', true)
            setVisibility('back', true)
            break;
        case 5:
            setVisibility('decision1', false)
            setVisibility('decision2', false)
            setVisibility('reset', true)
            setVisibility('back', true)
            break
        case 6:
            setVisibility('decision1', false)
            setVisibility('decision2', false)
            setVisibility('reset', true)
            setVisibility('back', true)
            break
        case 7:
            setVisibility('decision1', true)
            setVisibility('decision2', true)
            setVisibility('reset', true)
            setVisibility('back', true)
            break
        case 8:
            setVisibility('decision1', false)
            setVisibility('decision2', false)
            setVisibility('reset', true)
            setVisibility('back', true)
            break
        default:
            setVisibility('decision2', true)
            setVisibility('reset', true)
            setVisibility('back', true)
            break;
    }
}

const replaceNodeText = (targetElementID, message) => {
    const targetElement = document.getElementById(targetElementID)
    // remove children
    while (targetElement.firstChild) {
        targetElement.removeChild(targetElement.firstChild)
    }
    // update text
    targetElement.appendChild(document.createTextNode(message))
}
const changeScene = decision => {
    switch (curScene) {
        case 0:
            curScene = 1
            backScene = null
            message = 'Journey begins by taking a fork'
            replaceNodeText('decision1', 'take the path!')
            replaceNodeText('decision2', 'take the bridge')
            setControls(curScene)
            decisionStack.push([1, message])
            break
        case 1:
            if (decision == 1) {
                curScene = 2
                backDecision = 1
                backScene = 0
                message = 'You reached at a hut in the woods.'
                replaceNodeText('decision1', 'walk around back')
                replaceNodeText('decision2', 'knock on the door')
                setControls(curScene)
                decisionStack.push([1, message])
            } else if (decision == 2) {
                curScene = 3
                backScene = 0
                backDecision = 2
                message = 'You are standing on the bridge enjoying the water flow.'
                replaceNodeText('decision1', 'walk across bridge')
                replaceNodeText('decision2', 'gaze into stream')
                setControls(curScene)
                decisionStack.push([2, message])
            }
            break
        case 2:
            if (decision == 1) {
                curScene = 4
                backScene = 1
                backDecision = 1
                message = 'You see a witch in the hut'
                replaceNodeText('decision1', 'sneak by window')
                replaceNodeText('decision2', 'wave at witch')
                setControls(curScene)
                decisionStack.push([1, message])
            } else if (decision == 2) {
                curScene = 5
                backScene = 1
                backDecision = 1
                message = 'knocking the door, you are done!'
                replaceNodeText('decision1', 'game over')
                replaceNodeText('decision2', 'game over')
                setControls(curScene)
                decisionStack.push([2, message])
            }
            break
        case 3:
            if (decision == 1 ) {
                curScene = 7
                backScene = 1
                backDecision = 2
                message = 'You see a giant troll in front of you'
                replaceNodeText('decision1', 'say hello to stroll')
                replaceNodeText('decision2', 'Jump into stream')
                setControls(curScene)
                decisionStack.push([1, message])
            } else if (decision == 2) {
                curScene = 8
                backScene = 1
                backDecision = 2
                message = 'to be continued!'
                replaceNodeText('decision1', 'game over')
                replaceNodeText('decision2', 'game over')
                setControls(curScene)
                decisionStack.push([2, message])
            }
            break
        case 4:
            if (decision == 1) {
                curScene = 8
                backScene = 2
                backDecision = 1
                message = 'to be continued...'
                replaceNodeText('decision1', '')
                replaceNodeText('decision2', '')
                setControls(curScene)
                decisionStack.push([1, message])
            } else if (decision == 2) {
                backScene = 2
                backDecision = 1
                curScene = 8 
                message = 'to be continued...'
                setControls(curScene)
                decisionStack.push([2, message])
            }
            break
        case 5:
            message = 'will be updated soon. Play again?'
            curScene = 8
            break
        case 6:
            curScene = 8
            message = 'will be updated soon. Play again?'
            replaceNodeText('decision1', 'game over')
            replaceNodeText('decision2', 'game over')
            break
        case 7:
            if (decision == 1) {
                backScene = 3
                backDecision = 1
                curScene = 6
                message = 'got eaten by the troll!'
                replaceNodeText('decision1', '')
                replaceNodeText('decision2', '')
                setControls(curScene)
                decisionStack.push([1, message])
            } else if (decision == 2) {
                curScene = 8
                backScene = 1
                backDecision = 2
                message = 'drowned!'
                replaceNodeText('decision1', '')
                replaceNodeText('decision2', '')
                setControls(curScene)
                decisionStack.push([2, message])
            }
            break
        default:
            alert('unknown error!')
            reset()
            break;
    }
    setSceneImg(curScene)
    replaceNodeText('sceneText', message)
    setDecisionHistory()
}
const setDecisionHistory = () => {
    document.getElementById('history').innerHTML = null
    decisionStack.forEach(element => {
        let tag = document.createElement("p")
        let tagText = document.createTextNode(element[0] + ' - ' +element[1])
        tag.appendChild(tagText)
        document.getElementById('history').appendChild(tag)
    })
}
const back = (backScene, backDecision) => {
    if (backScene == null) {
        reset()
    } else {
        // todo: back does not work properly
        curScene = backScene
        // decisionStack.pop()
        changeScene(backDecision)
    }
    
}
const reset = () => {
    location.reload()
}

window.onload = event => {
    replaceNodeText('sceneText', 'You are standing alone in the woods.')
    replaceNodeText('decision1', 'Start Game!')
    replaceNodeText('decision2', '')
    setControls(curScene)
}