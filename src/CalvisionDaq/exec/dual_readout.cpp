#include "CalvisionDaq/common/BufferedFileIO.h"
#include "CalvisionDaq/digitizer/Digitizer.h"
#include "CalvisionDaq/digitizer/CaenError.h"
#include "CalvisionDaq/digitizer/Command.h"
#include "CalvisionDaq/common/BinaryIO.h"
#include "CalvisionDaq/digitizer/Decoder.h"
#include "CalvisionDaq/root/RootIO.h"
#include "CalvisionDaq/common/Stopwatch.h"
#include "CalvisionDaq/common/MemoryPool.h"
#include "CalvisionDaq/digitizer/Staging.h"

#include "CppUtils/c_util/CUtil.h"





#include <iostream>
#include <fstream>
#include <memory>
#include <atomic>

#include <iomanip>
#include <chrono>
std::ostream& log_with_timestamp(std::ostream& os) {
    using Clock = std::chrono::system_clock;
    auto now = Clock::now();
    auto now_time = Clock::to_time_t(now);
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) % 1000;

    os << "[" << std::put_time(std::localtime(&now_time), "%F %T")
       << "." << std::setw(3) << std::setfill('0') << millis.count()
       << "] ";
    return os;
}

//handling external kill signal
#include <csignal>    // for std::signal
extern std::atomic<bool> quit_readout;
void handle_sigterm(int signum) {
    std::cout << "[dual_readout] Caught signal " << signum << "stopping DAQ gracefully."<<std::endl;
    quit_readout.store(true);
}

std::atomic<bool> quit_readout = false;
std::atomic<bool> dump_hv = false;
std::atomic<bool> dump_lv = false;

using PoolType = MemoryPool<Digitizer::max_event_size()>;
using QueueType = SPSCQueue<typename PoolType::BlockType, 1024 * 50>;


void write_event_to_dat_file_raw(const x742EventData& event, std::ofstream& out) {
    int32_t event_number = event.event_counter;
    out.write(reinterpret_cast<const char*>(&event_number), sizeof(event_number));
    
    // Write raw ADC values as int16_t
    for (int ch = 0; ch < 16; ++ch) {
        const auto& group = event.group_data[ch / 8];
        const auto& waveform = group.channel_data[ch % 8]; // Use raw ADC data here
        for (auto sample : waveform) {
            int16_t adc_val = static_cast<int16_t>(sample);
            out.write(reinterpret_cast<const char*>(&adc_val), sizeof(adc_val));
        }
    }

    const auto& trig_waveform = event.group_data[0].trigger_data;
    for (auto sample : trig_waveform) {
        int16_t trig_adc = static_cast<int16_t>(sample);
        out.write(reinterpret_cast<const char*>(&trig_adc), sizeof(trig_adc));
    }
}

void write_event_to_dat_file_corrected(const x742EventData& event, std::ofstream& out) {
    int32_t event_number = event.event_counter;
    out.write(reinterpret_cast<const char*>(&event_number), sizeof(event_number));

    // Write calibrated float waveform
    for (int ch = 0; ch < 16; ++ch) {
        const auto& group = event.group_data[ch / 8];
        const auto& waveform = group.channel_data[ch % 8]; // Corrected waveform
        for (auto sample : waveform) {
            float val = sample;  // calibrated float
            out.write(reinterpret_cast<const char*>(&val), sizeof(val));
        }
    }

    const auto& trig_waveform = event.group_data[0].trigger_data;
    for (auto sample : trig_waveform) {
        float trig_val = sample;
        out.write(reinterpret_cast<const char*>(&trig_val), sizeof(trig_val));
    }
}


void decode_loop(size_t thread_id, DigitizerContext& ctx, PoolType& pool, QueueType& queue, std::atomic<bool>& dump) {
    std::cout << "[dual_readout]Thread"<<thread_id << ": Creating decoder"<< std::endl;
    Decoder decoder(ctx.digi().serial_code());
    const std::string root_io_path = ctx.path_prefix() + "/outfile_" + ctx.name() + ".root";
    std::cout << "[dual_readout]Thread"<<thread_id << ": Creating root io: " << root_io_path << std::endl;
    RootWriter root_io(root_io_path);
    root_io.setup(decoder.event());

    std::cout << "[dual_readout]Thread"<<thread_id << ": Entering decode loop\n";

    // create .dat output stream
    std::ofstream dat_out_raw(ctx.path_prefix() + "/outfile_raw_" + ctx.name() + ".dat", std::ios::binary);
    std::ofstream dat_out_corrected(ctx.path_prefix() + "/outfile_corrected_" + ctx.name() + ".dat", std::ios::binary);
    if (!dat_out_raw.is_open() || !dat_out_corrected.is_open()) {
        std::cerr << "Failed to open dat files for writing\n";
        return;
    }


    Stopwatch<std::chrono::microseconds> stopwatch;
    // uint64_t last_read_waits = 0;

    size_t decode_count = 0;
    // bool written = false;

    while (auto block = queue.pop()) {
        // std::cout << "Decoding event " << decode_count << std::endl;
        // std::cout << thread_id << ": Got a block!\n";

        // const auto wait_duration = stopwatch();
        // if (ctx.queue().read_waits() > last_read_waits) {
        //     std::cout << thread_id << ": read waited " << wait_duration << std::endl;
        //     last_read_waits = ctx.queue().read_waits();
        // }

        BinaryInputBufferStream input((const char*) block->block().data(), ctx.digi().event_size());

        // std::cout << "Expected event size: " << ctx.digi().event_size() << std::endl;

        // Write raw dat file
        write_event_to_dat_file_raw(decoder.event(), dat_out_raw);
        


        decoder.read_event(input);
        decoder.apply_corrections();

        //Write corrected dat file
        // if(!written){
        write_event_to_dat_file_corrected(decoder.event(), dat_out_corrected);
            // written = true;
        // }

        
        // stopwatch();

        // Write ROOT file
        root_io.handle_event(decoder.event());
        // std::cout << thread_id << ": Root io: " << stopwatch() << std::endl;
        if (dump.load()) {

            // // --- Overflow Check --- not working
            // uint32_t status = 0;
            // int err = CAEN_DGTZ_ReadRegister(ctx.digi().handle(), 0x812C, &status);
            // if (err != 0) {
            //     std::cout << "[dual_readout]Thread"<<thread_id << ": [ERROR] Failed to read Acquisition Status Register: " << err << std::endl;
            // } else if (status & (1 << 2)) {
            //     std::cout << "[dual_readout]Thread"<<thread_id << ": [WARNING] Buffer overflow detected!" << std::endl;
            // } else{
            //     std::cout << "[dual_readout]Thread"<<thread_id << "Digitizer buffer is not overflowed." << std::endl;
            // }

            // std::cout << "[dual_readout]Thread"<<thread_id << ": writing waveform dump"<< std::endl;;
            root_io.dump_last_event(ctx.path_prefix() + "/dump_" + ctx.name());
            dump.store(false);
            // std::cout << "[dual_readout]Thread"<<thread_id << ": waveform dump written"<< std::endl;;
        }

        // std::cout << thread_id << ": Deallocating block " << block << std::endl;

        pool.deallocate(block);

        // std::cout << thread_id << ": Deallocated.\n";

        decode_count++;
        if (ctx.digi().max_readout_count() && decode_count >= *ctx.digi().max_readout_count()) {
            break;
        }

        // stopwatch();
    }

    root_io.write();
    std::cout << "[dual_readout]Thread"<<thread_id << ": decode_loop exiting. decode_count=" << decode_count << std::endl;

}

void main_loop(size_t thread_id, DigitizerContext& ctx, std::atomic<bool>& dump) {

    PoolType pool(1024);
    QueueType queue;

    // static int log_count = 0;
    ctx.digi().set_event_callback(
        [&pool, &queue, thread_id]
        (const char* data, UIntType event_size, UIntType num_events) {
            auto blocks = pool.allocate(num_events);
            for (auto* block : blocks) {
                copy_raw_buffer<uint8_t, char>(block->block().data(), data, event_size);
                queue.add(block);
                // if (log_count < 10) {
                //     std::cout << "[readout_thread] block pushed to decode queue" << thread_id << std::endl;
                //      ++log_count;
                // }
                data += event_size;
            }
        });


    std::thread decode_thread(&decode_loop, thread_id, std::ref(ctx), std::ref(pool), std::ref(queue), std::ref(dump));

    try {

        log_with_timestamp(ctx.log()) << "Beginning readout" << std::endl;
        static UIntType last = -1;
        
        ctx.digi().readout([](const Digitizer& d) {
                UIntType current = d.num_events_read();
                if (current != last && (current <= 10 || (current <= 100 && current % 10 == 0) || (current <= 1000 && current % 100 == 0) || (current % 1000 == 0))) {
                    std::cout << "Read " << current << std::endl;
                    last = current;
                }
                // std::cout.flush();

                // Can maybe sleep for more efficient readouts?
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(5ms);
                // std::cout << "Woke from sleep\n";
                // std::cout.flush();
                if (d.max_readout_count() && d.num_events_read() >= *d.max_readout_count()) {
                    return false;
                }
                // std::cout << "Finished predicate check\n";
                // std::cout.flush();
                return !quit_readout.load();
            });


        log_with_timestamp(ctx.log()) << thread_id << ": Stopped readout" << std::endl;

        queue.close();
        decode_thread.join();

        log_with_timestamp(ctx.log()) << thread_id << ": Resetting digitzer..." << std::endl;
        ctx.digi().reset();

        log_with_timestamp(ctx.log()) << thread_id << ": Done." << std::endl;

    } catch (const CaenError& error) {
        std::cerr << "[FATAL ERROR]: ";
        error.print_error(std::cerr);
        error.print_error(log_with_timestamp(ctx.log()));
        quit_readout.store(true);;
        queue.close();
    } catch(...) {
        std::cerr << "[UNEXPECTED FATAL ERROR!]\n";
        quit_readout.store(true);
        queue.close();
    }

    if (decode_thread.joinable()) {
        decode_thread.join();
    }
}

#include <termios.h>

void interrupt_listener() {
    std::cout << "[interrupt_listener] Starting input loop\n";

    struct termios cintty;
    struct termios cinsave;
    tcgetattr(STDIN_FILENO, &cinsave);
    cintty = cinsave;

    cintty.c_lflag &= ~(ICANON);
    cintty.c_cc[VMIN] = 0;
    cintty.c_cc[VTIME] = 0;
    cintty.c_cc[VEOF] = 4;

    tcsetattr(STDIN_FILENO, TCSANOW, &cintty);

    char buffer[4096];
    size_t current_size = 0;

    while (!quit_readout.load()) {
        // std::cout << "[interrupt_listener] Polling stdin..."<<std::endl;
        int num_read = ::read(STDIN_FILENO, buffer + current_size, 4095 - current_size);
        
        if (num_read < 0) {
            std::cerr << "[interrupt_listener] Error reading stdin"<<std::endl;
            quit_readout.store(true);
            break;
        } else if (num_read == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        current_size += num_read;
        buffer[current_size] = '\0';
        std::string message(buffer);

        // std::cout << "[interrupt_listener] Received input: " << message <<std::endl;

        if (message == "stop\n") {
            std::cout << "[interrupt_listener] Stop command received"<<std::endl;
            quit_readout.store(true);
            break;
        } else if (message == "sample plot\n") {
            dump_hv.store(true);
            // std::cout << "[interrupt_listener] Sample plot command received"<<std::endl;
            current_size = 0;
        }
    }

    std::cout << "[interrupt_listener] Exiting"<<std::endl;
    tcsetattr(STDIN_FILENO, TCSANOW, &cinsave);
}



template <DigiMap d>
DigitizerContext& make_context(AllDigitizers& digis, char** argv) {

    auto* ctx = digis.get<d>();
    if (!ctx) {
        std::cerr << "Couldn't find "
                  << DigiMapTable.get<d, DigiMapValue::Name>()
                  << " digitizer (serial number "
                  << DigiMapTable.get<d, DigiMapValue::Serial>()
                  << ")\n";
        std::exit(1);
    }

    ctx->make_log(std::string(DigiMapTable.get<d, DigiMapValue::Name>()));
    ctx->digi().load_config(std::string(argv[DigiMapTable.get<d, DigiMapValue::ArgvNum>()]));
    ctx->digi().print();
    return *ctx;
}

#include <TROOT.h>

int main(int argc, char** argv) {
    std::signal(SIGTERM, handle_sigterm);
    std::signal(SIGINT,  handle_sigterm); // for Ctrl+C or terminal closes



    if (argc != 4) {
        std::cout << "Usage: " << argv[0] << " [run name] [config hv] [config lv]\n";
        return 1;
    }

    ROOT::EnableThreadSafety();

    std::vector<std::thread> main_threads;
    std::thread listener(&interrupt_listener);

    std::string log_path;

    try {
        AllDigitizers digis{std::string(argv[1])};

        auto& hv_ctx = make_context<DigiMap::HG>(digis, argv);
        log_path = hv_ctx.path_prefix() + "/readout_" + hv_ctx.name() + ".log";

        // auto& lv_ctx = make_context<DigiMap::LG>(digis, argv);

        std::cout << "Beginning to make threads...\n";

        main_threads.emplace_back(&main_loop, 1, std::ref(hv_ctx), std::ref(dump_hv));
        // main_threads.emplace_back(&main_loop, 2, std::ref(lv_ctx), std::ref(dump_lv));


        // joining threads

        for (size_t i = 0; i < main_threads.size(); i++) {
            try {
                main_threads[i].join();
            } catch (const CaenError& error) {
                std::cerr << "[FATAL INTERIOR ERROR]: " ;
                error.print_error(std::cerr);
                quit_readout.store(true);
            }
        }

    } catch(const CaenError& error) {
        std::cerr << "[FATAL ERROR]: ";
        error.print_error(std::cerr);
    }


    quit_readout.store(true);

    listener.join();
    std::ofstream append_log(log_path, std::ios::app);
    if (append_log) {
        log_with_timestamp(append_log)
            << "Clean exit confirmed."
            << std::endl;
    } else {
        std::cerr << "[ERROR] Failed to reopen log file for appending: " << log_path << std::endl;
    }


    return 0;
}
