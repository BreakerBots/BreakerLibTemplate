// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class FalconOrchestra extends SubsystemBase {
    private Orchestra orchestra;
    private String[] currentPlaylist;
    private int nextPlaylistSong = 0;
    private boolean runPlaylist = false;
    public FalconOrchestra() {
        orchestra = new Orchestra();
    }
    
    public void startPlaylist(String[] playlistSongFilepaths) {
        currentPlaylist = playlistSongFilepaths;
        runPlaylist = true;
    }

    public void stopPlaylist() {
        runPlaylist = false;
        nextPlaylistSong = 0;
    }

    public void stopMusic() {
        orchestra.stop();
    }

    public void pauseMusic() {
        orchestra.pause();
    }

    public void playMusic() {
        orchestra.play();
    }

    public void loadMusic(String musicFilepath) {
        orchestra.loadMusic(musicFilepath);
        stopPlaylist();
    }

    public boolean isPaused() {
        return !(orchestra.getCurrentTime() == 0) && !orchestra.isPlaying();
    }

    public boolean isStoped() {
        return (orchestra.getCurrentTime() == 0) && !orchestra.isPlaying();
    }

    @Override
    public void periodic() {
        if (runPlaylist && isStoped()) {
            try {
                orchestra.loadMusic(currentPlaylist[nextPlaylistSong]);
                orchestra.play();
                nextPlaylistSong ++;
            } catch (Exception e) {
                nextPlaylistSong = 0;
                orchestra.stop();
            }
            
        }
    }
}
