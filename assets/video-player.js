/* Native video controls remain usable without JavaScript. */
document.querySelectorAll('.film').forEach((film) => {
  const video = film.querySelector('video');
  const button = film.querySelector('.play-video');
  const status = film.querySelector('.video-status');
  const alternate = film.querySelector('.use-webm');
  let fallbackTried = false;

  const reportError = () => {
    const code = video.error?.code;
    status.textContent = code === 2
      ? 'The video could not finish loading. Try the alternative format or open the video below.'
      : 'Playback is unavailable in this browser. Try WebM or open the video below.';
    status.hidden = false;
    button.hidden = false;
  };

  const play = async () => {
    status.hidden = true;
    button.disabled = true;
    try {
      await video.play();
    } catch (error) {
      if (error.name === 'NotSupportedError' && !fallbackTried) {
        fallbackTried = true;
        video.src = video.dataset.webm;
        video.load();
        try { await video.play(); } catch (_) { reportError(); }
      } else if (error.name !== 'AbortError') {
        reportError();
      }
    } finally {
      button.disabled = false;
    }
  };

  button.hidden = false;
  button.addEventListener('click', play);
  alternate.hidden = false;
  alternate.addEventListener('click', () => {
    fallbackTried = true;
    video.src = video.dataset.webm;
    video.load();
    play();
  });
  video.addEventListener('play', () => {
    button.hidden = true;
    status.hidden = true;
    document.querySelectorAll('.film video').forEach((other) => {
      if (other !== video) other.pause();
    });
  });
  video.addEventListener('pause', () => {
    button.textContent = video.ended ? '▶ Replay video' : '▶ Resume video';
    button.hidden = false;
  });
  video.addEventListener('ended', () => {
    button.textContent = '▶ Replay video';
    button.hidden = false;
  });
  video.addEventListener('error', reportError);
});
