// Record `seconds` of the default microphone to a 16 kHz mono int16 wav (macOS, AVFoundation).
// Used by record_real.py (compiled on first use with swiftc). usage: record_mic OUT.wav SECONDS
import AVFoundation
import Foundation

let args = CommandLine.arguments
guard args.count == 3, let seconds = Double(args[2]) else {
    FileHandle.standardError.write("usage: record_mic OUT.wav SECONDS\n".data(using: .utf8)!)
    exit(2)
}
let settings: [String: Any] = [
    AVFormatIDKey: Int(kAudioFormatLinearPCM), AVSampleRateKey: 16000.0, AVNumberOfChannelsKey: 1,
    AVLinearPCMBitDepthKey: 16, AVLinearPCMIsFloatKey: false, AVLinearPCMIsBigEndianKey: false,
]
do {
    let rec = try AVAudioRecorder(url: URL(fileURLWithPath: args[1]), settings: settings)
    guard rec.record(forDuration: seconds) else {
        FileHandle.standardError.write("microphone access denied or unavailable\n".data(using: .utf8)!)
        exit(1)
    }
    Thread.sleep(forTimeInterval: seconds + 0.3)
    rec.stop()
} catch {
    FileHandle.standardError.write("record failed: \(error)\n".data(using: .utf8)!)
    exit(1)
}
